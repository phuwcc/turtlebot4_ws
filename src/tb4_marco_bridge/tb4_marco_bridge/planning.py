"""Physical-robot adapter to MARCO's unmodified, simulator-free core."""
import hashlib
import heapq
import importlib
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path


def finite(value):
    return type(value) in (float, int) and math.isfinite(value)


def load_core(root):
    root = Path(root).expanduser().resolve()
    if not (root/'src/core/coordination/relocation_search.py').is_file():
        raise ValueError('MARCO_ROOT must point to the MACRO checkout')
    # No __pycache__, build metadata, scenario outputs or logs inside MARCO.
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(root/'src'))
    return importlib.import_module('core')


def robot_index(name):
    """Fleet robot IDs are robotN with N >= 1."""
    if not isinstance(name,str) or not name.startswith('robot') or not name[5:].isdigit() or int(name[5:])<1:
        raise ValueError('Unsupported robot identifier: '+repr(name))
    return int(name[5:])


@dataclass
class RobotState:
    """Simulator-independent snapshot consumed by MACRO's coordination core."""
    robot_id: str
    cell: tuple
    color: tuple
    goal: tuple | None = None
    path: list = field(default_factory=list)
    path_index: int = 0
    phase: str = 'idle'
    current_task: dict | None = None
    action_remaining: float = 0.
    interrupted_action_remaining: float | None = None


class Grid:
    def __init__(self, message, radius):
        self.message = message
        self.width, self.height = message['width'], message['height']
        self.res = message['resolution']
        self.origin = message['origin']
        if (type(self.width) is not int or type(self.height) is not int
                or self.width <= 0 or self.height <= 0 or not 0 < self.width*self.height <= 1_000_000
                or not finite(self.res) or self.res <= 0
                or len(message['data']) != self.width*self.height
                or len(self.origin) != 3 or not all(finite(v) for v in self.origin)):
            raise ValueError('Invalid occupancy grid')
        self.signature = hashlib.sha256(json.dumps(message, sort_keys=True).encode()).hexdigest()
        # Euclidean distance to occupied/unknown cell centres, with map boundary
        # and half-cell diagonal included. Never silently snap an unsafe goal.
        w, h = self.width, self.height
        blocked = {(x,y) for y in range(h) for x in range(w)
                   if message['data'][y*w+x] != 0 or x in (0,w-1) or y in (0,h-1)}
        reach = radius/self.res + math.sqrt(.5)
        offsets = [(dx,dy) for dx in range(-math.ceil(reach), math.ceil(reach)+1)
                   for dy in range(-math.ceil(reach), math.ceil(reach)+1)
                   if math.hypot(dx,dy) <= reach]
        forbidden = set(blocked)
        for x,y in blocked:
            forbidden.update((x+dx,y+dy) for dx,dy in offsets)
        self.cells = {(x,y) for y in range(h) for x in range(w) if (x,y) not in forbidden}
        self.cos, self.sin = math.cos(self.origin[2]), math.sin(self.origin[2])

    def cell_coordinates(self, pose):
        x,y = pose[0]-self.origin[0], pose[1]-self.origin[1]
        return (self.cos*x+self.sin*y)/self.res, (-self.sin*x+self.cos*y)/self.res

    def cell(self, pose):
        return tuple(map(math.floor, self.cell_coordinates(pose)))

    def continuous_cell(self, pose):
        # MACRO indexes cell centres at integers. Keep a parked peer's measured
        # centre precise even when its pose straddles a grid boundary.
        x,y = self.cell_coordinates(pose)
        return x-.5, y-.5

    def world(self, cell):
        x,y = (cell[0]+.5)*self.res, (cell[1]+.5)*self.res
        return [self.origin[0]+self.cos*x-self.sin*y, self.origin[1]+self.sin*x+self.cos*y]

    def points(self, path, initial, goal=None):
        values = [list(initial[:2])] + [self.world(c) for c in path[1:]]
        if len(values)==1:
            values.append(list(goal[:2]) if goal is not None else list(values[0]))
        elif goal is not None:
            values[-1] = list(goal[:2])
        result=[]
        for i, p in enumerate(values):
            a,b = (p,values[i+1]) if i+1<len(values) else (values[max(0,i-1)],p)
            angle = math.atan2(b[1]-a[1], b[0]-a[0])
            if i==len(values)-1 and goal is not None and len(goal)==3:
                angle=goal[2]
            result.append([*p,angle])
        if len(result)>512:
            raise ValueError('Route exceeds 512 points; split the mission into shorter tasks')
        return result


class Planner:
    def __init__(self, root):
        self.core=load_core(root)
        from core.coordination import planner, relocation_search
        self.algorithms, self.coordinator = planner, relocation_search
        self.conflict=relocation_search._path_conflict
        # Use MACRO's own robot snapshot type so every core/standalone routine
        # (local resolution timing, release refiner) sees the attributes it expects.
        try:
            from simulations.standalone.app import RobotState as MacroRobotState
            self.RobotState=MacroRobotState
        except ImportError:
            self.RobotState=RobotState
        self.grid=None

    def prepare(self, snapshot, mission):
        if not isinstance(mission,dict) or not isinstance(mission.get('tasks'),list) or not 0 < len(mission['tasks']) <= 16:
            raise ValueError('Mission needs 1..16 tasks')
        # MARCO mode passes MACRO's own coordination radius (marco_geometry.ROBOT_RADIUS,
        # 0.22 m in the ROS/grid simulations). Without it, fall back to the
        # conservative adapter composition used by CLI missions.
        self.marco_radius=mission.get('coordination_radius')
        if self.marco_radius is not None:
            self.radius=self.marco_radius
        else:
            self.radius=mission.get('robot_radius',.175)+mission.get('safety_margin',.05)+mission.get('tracking_tolerance',.04)
        self.speed=mission.get('speed_mps',.05)
        if not finite(self.radius) or not .175 <= self.radius <= .6 or not finite(self.speed) or not 0 < self.speed <= .25:
            raise ValueError('Invalid robot clearance or speed (maximum 0.25m/s)')
        if not self.grid or self.grid.message != snapshot['map'] or self.last_radius != self.radius:
            self.grid=Grid(snapshot['map'],self.radius);self.last_radius=self.radius
        self.robots=tuple(sorted(snapshot['robots'],key=robot_index))
        self.priority_order=mission.get('priority_order')
        if self.priority_order is not None and (not isinstance(self.priority_order,list) or
                len(self.priority_order)!=len(self.robots) or set(self.priority_order)!=set(self.robots)):
            raise ValueError('priority_order must list every active robot exactly once')
        if not self.robots:
            raise ValueError('No robots in snapshot')
        # Fleet robot "robotN" is MACRO robot "RN" (any N).
        self.ids={r:'R'+str(robot_index(r)) for r in self.robots};self.names={v:k for k,v in self.ids.items()}
        self.positions={self.ids[r]:self.grid.cell(snapshot['robots'][r]['pose']) for r in self.robots}
        if any(c not in self.grid.cells for c in self.positions.values()):
            raise ValueError('A robot footprint overlaps unknown/occupied map cells; check localization')
        # MACRO scenario objects, given in metric map coordinates instead of grid cells.
        objects={}
        for item in mission.get('objects',[]):
            pose=item.get('pose') if isinstance(item,dict) else None
            if not isinstance(item,dict) or not isinstance(item.get('name'),str) or not isinstance(pose,list) \
                    or len(pose) not in (2,3) or not all(finite(v) for v in pose):
                raise ValueError('Each object needs a name and a finite map pose [x,y] or [x,y,yaw]')
            objects[item['name']]=list(pose)
        self.goals={};self.tasks=[];self.durations={}
        # MACRO object table (name -> grid cell): scenario objects keep their real
        # names so MACRO's DAG model sees e.g. a picked kit carried to its target.
        self.object_cells={name:self.grid.cell(pose) for name,pose in objects.items()}
        for t in mission['tasks']:
            if not isinstance(t,dict) or type(t.get('id')) is not int or t['id']<=0:
                raise ValueError('Each task needs a unique positive integer id')
            candidates=t.get('candidate',t.get('candidates',list(self.robots)))
            dependencies=t.get('node',t.get('depends_on',[]))
            action=t.get('action','navigate');duration=t.get('duration',.001)
            if 'goal' in t:
                goal=t['goal']
            else:
                # MACRO (standalone/app.py _destination_name): a place goes to its target.
                destination=t.get('target') if action=='place' else t.get('object')
                if destination not in objects:
                    raise ValueError(f'Task {t["id"]}: unknown object/target {destination!r}')
                goal=objects[destination]
            if not isinstance(goal,list) or len(goal) not in (2,3) or not all(finite(v) for v in goal):
                raise ValueError('Task goal must be finite [x,y] or [x,y,yaw] in map')
            candidates=[self.names.get(r,r) for r in candidates] if isinstance(candidates,list) else candidates
            if not isinstance(candidates,list) or not candidates or any(r not in self.ids for r in candidates):
                raise ValueError(f'Task {t["id"]}: candidates must reference active robots')
            if not isinstance(dependencies,list) or any(type(v) is not int for v in dependencies):
                raise ValueError('node/depends_on must contain task IDs')
            if not finite(duration) or duration<=0:
                raise ValueError(f'Task {t["id"]}: duration must be positive seconds')
            cell=self.grid.cell(goal)
            if cell not in self.grid.cells:raise ValueError(f'Task {t["id"]}: goal is not footprint-safe in the map')
            self.goals[str(t['id'])]=(cell,goal);self.durations[t['id']]=float(duration)
            item={'id':t['id'],'candidate':[self.ids[r] for r in candidates],'node':dependencies,
                  'action':action,'duration':float(duration)}
            if 'goal' in t:
                item['object']=str(t['id']);self.object_cells[str(t['id'])]=cell
            else:
                item['object']=t['object']
                if action=='place':item['target']=t['target']
            self.tasks.append(item)
        self.core.validate_tasks(self.tasks)
        self._mission=mission

    def assign(self, snapshot, mission):
        self.prepare(snapshot,mission)
        parent=self
        class Distance:
            def plan(self,robot,location,start_position=None):
                rid=parent.ids[robot]
                start=start_position if start_position is not None else parent.positions[rid]
                goal=parent.object_cells[location]
                route=parent.algorithms.astar(parent.grid.cells,start,goal)
                if route is None:raise ValueError('No static route for task '+location)
                return parent.algorithms.path_length(route,parent.grid.res),goal
        from core.assignment_cpp import require_native
        require_native()
        mode='chain_bnb_cpp'
        # service_time=None: MACRO reads each task's own duration.
        result=self.core.optimize_task_assignments(self.tasks,Distance(),robot_speed=self.speed,service_time=None,search_mode=mode)
        return {'assignment':{str(k):self.names[v] for k,v in result['assignment'].items()},
                'sequences':{self.names[k]:v for k,v in result['robot_plans'].items()},
                'estimated_makespan_s':result['makespan'],'total_distance_m':result['total_distance'],
                'backend':mode,'map_sha256':self.grid.signature,'optimality_claim':False}

    def next_batch(self, snapshot, mission, assignment, completed, intent=None):
        self.prepare(snapshot,mission)
        assigned={int(k):self.ids[v] for k,v in assignment['assignment'].items()}
        completed=set(completed);current={};tasks=[]
        for t in self.tasks:
            item=dict(t,robot=assigned[t['id']]);tasks.append(item)
        task_by_id={t['id']:t for t in tasks}
        for name,sequence in assignment['sequences'].items():
            remaining=[tid for tid in sequence if tid not in completed]
            if remaining and set(task_by_id[remaining[0]]['node'])<=completed:
                current[self.ids[name]]=task_by_id[remaining[0]]
        if not current:return {'actions':[],'reason':'completed' if len(completed)==len(tasks) else 'waiting_for_dependencies'}
        states=[];routes={}
        # Additional half-cell protection covers centre quantization at execution.
        radius=self.dynamic_radius()
        for rid,start in self.positions.items():
            task=current.get(rid);goal=self.goals[str(task['id'])][0] if task else None
            route=self.algorithms.astar(self.grid.cells,start,goal) if task else [start]
            if route is None:raise ValueError('No route from current robot position')
            routes[rid]=route
            states.append(self.RobotState(rid,start,(0,0,0),goal=goal,path=route,phase='preview' if task else 'idle',current_task=task))
        clearance=2*radius/self.grid.res
        def drive_route(rid,goal):
            """Driving is the lower layer: A* from the robot's cell to the goal that
            MARCO (or the task) chose, around the bodies of every other robot."""
            centres=[(self.positions[o] if self.marco_radius is not None else
                      self.grid.continuous_cell(snapshot['robots'][self.names[o]]['pose']))
                     for o in self.positions if o!=rid]
            return self.algorithms._astar_around_robot_bodies(self.grid.cells,self.positions[rid],goal,
                robot_centres=centres,clearance_cells=clearance)
        if intent:
            priority=intent['priority'];tid=intent['task']
            if priority not in current or current[priority]['id']!=tid:
                raise RuntimeError('Committed intervention no longer matches ready DAG tasks')
            remaining=intent.get('relocations',[])
            if remaining:
                step=remaining[0];rid=step['robot'];goal=tuple(step['destination']);task_id=None
                next_intent={**intent,'relocations':remaining[1:]}
            else:
                rid=priority;goal=self.goals[str(tid)][0];task_id=tid;next_intent=None
            # MARCO fixed only the temporary holding goal; the move itself is A*.
            route=drive_route(rid,goal)
            if route is None:
                # FollowPath can finish within its goal tolerance while leaving
                # a peer inside the clearance assumed by the old intervention.
                # Search again from the measured poses and keep mission goals fixed.
                batch = self.next_batch(snapshot, mission, assignment, completed)
                batch['audit']['replanned_committed_path'] = True
                return batch
            return self.package_actions(snapshot,[(rid,route,task_id)],{'strategy':'committed_clearance' if remaining else 'committed_priority'},next_intent)
        if self.priority_order and len(current)==1:
            rid=next(iter(current));robot=self.names[rid]
            fixed=self.priority_order[:self.priority_order.index(robot)]
            if fixed:
                task=current[rid]
                route=self.algorithms._astar_around_robot_bodies(self.grid.cells,self.positions[rid],
                    self.goals[str(task['id'])][0],
                    robot_centres=[self.grid.continuous_cell(state['pose']) for name,state in snapshot['robots'].items()
                                   if name!=robot],clearance_cells=clearance)
                if route is not None:
                    return self.package_actions(snapshot,[(rid,route,task['id'])],
                        {'strategy':'fixed_priority_detour','priority_robot':rid,'held_robots':[self.ids[r] for r in fixed]},None)
                if all(r in fixed for r in self.robots if r!=robot):
                    raise RuntimeError('No route without displacing the higher-priority robot')
                # An idle lower-priority robot (MACRO three_robot_t R3) blocks the
                # way: let the MARCO relocation search move it, holding `fixed`.
                blocked_by_idle=True
            else:blocked_by_idle=False
        else:blocked_by_idle=False
        # Protect complete routes, including stationary peer footprints, rather
        # than assuming a simulator's predicted arrival times are exact.
        conflicts=blocked_by_idle or any(self.conflict(routes[a],routes[b],clearance) for i,a in enumerate(routes) for b in list(routes)[i+1:])
        if conflicts and self._mission.get('nominal_until_contact') and not self._mission.get('intervene') and len(current)>=1:
            # MARCO ROS execution (three_robot_t/execute.py): every robot first
            # drives its nominal route; the executor watches predicted swept
            # contacts and only then stops the robots and requests the MARCO
            # intervention from the actual conflict poses (mission 'intervene').
            # Idle robots hold position and take part in that swept trigger, so
            # a route passing a parked idle robot is not rejected up front.
            return self.package_actions(snapshot,[(r,routes[r],current[r]['id']) for r in sorted(current)],
                {'strategy':'nominal','route_conflict':True},None,guard_peers=False)
        audit={'strategy':'disjoint_routes'}
        selected=[(r,routes[r],current[r]['id']) for r in current]
        next_intent=None
        if conflicts:
            objects={name:{'cell':cell,'cells':{cell},'interaction_cells':{cell},'blocking':False} for name,cell in self.object_cells.items()}
            def timing(candidate,*_):
                # Conservative metric timing supplied by this physical adapter.
                steps=candidate.get('relocation_sequence',[])
                distance=sum(self.algorithms.path_length(s['path'],self.grid.res) for s in steps)
                priority=candidate['priority_robot']
                route=candidate['preferred_paths'].get(priority,[])
                return distance/self.speed+self.algorithms.path_length(route,self.grid.res)/self.speed+.5*(len(steps)+1)
            task_status={t['id']:'completed' if t['id'] in completed else 'pending' for t in tasks}
            if not self.priority_order:
                # MARCO as in MACRO (standalone/app.py, used by three_robot_t): every
                # robot is in the cluster with the whole-map DAG state; every active
                # worker is a priority root H(r_p); reroute candidates, external
                # protected paths and the release-event refiner rank the branches,
                # and dag_aware selection picks which robot passes first.
                from simulations.standalone.app import _compute_conflict_plan
                plan=_compute_conflict_plan(cluster=states,dag_robots=states,tasks=tasks,task_status=task_status,
                    objects=objects,navigation_cells=self.grid.cells,robot_radius=radius,grid_size=self.grid.res,
                    speed_cells_per_second=self.speed/self.grid.res,selection_policy='dag_aware',
                    planning_timeout_s=float(self._mission.get('planning_timeout_s',10.)),
                    release_prediction_model='dynamic_events_guarded')
            else:
                # Legacy fixed-priority mode (CLI missions with priority_order).
                active_priority=next(r for r in self.priority_order if self.ids[r] in current)
                plan=self.coordinator.search_relocation_plan(robots=states,dag_robots=states,tasks=tasks,
                    task_status=task_status,objects=objects,
                    navigation_cells=self.grid.cells,robot_radius=radius,grid_size=self.grid.res,
                    speed_cells_per_second=self.speed/self.grid.res,planning_timeout_s=3.,
                    local_time_evaluator=timing,selection_policy='dag_aware',
                    priority_robot_ids=[self.ids[active_priority]],
                    fixed_robot_ids=[self.ids[r] for r in self.priority_order[:self.priority_order.index(active_priority)]])
                if plan and plan['priority_robot']!=self.ids[active_priority]:
                    raise RuntimeError('MARCO returned a plan violating the fixed priority')
            if not plan:raise RuntimeError('No feasible guarded MARCO intervention')
            audit={key:plan.get(key) for key in ('strategy','selected_hypothesis','priority_robot','local_resolution_delay','estimated_makespan')}
            relocations=plan.get('relocation_sequence',[])
            # Use MARCO's decision (priority branch + ordered temporary holding
            # goals); each move is driven by A* to that goal, not MARCO's grid path.
            audit['holding_goals']=[{'robot':self.names[item['robot_id']],
                'goal':list(self.grid.world(tuple(item['path'][-1]))[:2])} for item in relocations]
            if relocations:
                step=relocations[0];rid=step['robot_id'];goal=tuple(step['path'][-1])
                route=drive_route(rid,goal)
                if route is None:raise RuntimeError(f'No A* route to MARCO holding goal for {self.names[rid]}')
                selected=[(rid,route,None)]
                priority=plan['priority_robot']
                next_intent={'priority':priority,'task':current[priority]['id'],
                    'relocations':[{'robot':item['robot_id'],'destination':list(item['path'][-1])} for item in relocations[1:]]}
            else:
                rid=plan['priority_robot'];route=drive_route(rid,self.goals[str(current[rid]['id'])][0])
                if route is None:raise RuntimeError(f'No A* route for priority robot {self.names[rid]}')
                selected=[(rid,route,current[rid]['id'])]
        return self.package_actions(snapshot,selected,audit,next_intent)

    def dynamic_radius(self):
        if self.marco_radius is not None:
            return self.radius  # identical clearance to MACRO's planner and executor
        # Static inflation already includes cell area. For peer paths account for
        # residual grid quantization without charging the same margin twice.
        # The exact metric segment audit below independently enforces physical clearance.
        return self.radius+max(0.,math.sqrt(.5)*self.grid.res-self._mission.get('safety_margin',.05)+.01)

    def package_actions(self,snapshot,selected,audit,intent,guard_peers=True):
        actions=[]
        clearance=2*self.dynamic_radius()/self.grid.res
        # MARCO verifies clearance between grid-cell centres; a measured pose sits
        # up to half a cell diagonal from its centre. Bodies (0.35 m) still keep
        # >= 5 cm apart at this threshold.
        physical=(2*self.radius-math.sqrt(.5)*self.grid.res if self.marco_radius is not None else
                  2*(self._mission.get('robot_radius',.175)+self._mission.get('tracking_tolerance',.04))+.02)
        for rid,route,tid in selected:
            # Nominal MARCO batches are guarded at run time by the swept-contact
            # trigger instead of these static held-peer checks.
            # In MARCO mode check against the same cells MARCO planned with, so a
            # sub-cell pose offset cannot veto a plan MARCO already verified.
            peers=[(self.positions[self.ids[name]] if self.marco_radius is not None else self.grid.continuous_cell(state['pose']))
                   for name,state in snapshot['robots'].items()
                   if guard_peers and self.ids[name] not in {item[0] for item in selected}]
            for peer in peers:
                previous=math.dist(route[0],peer);cleared=previous>clearance
                for cell in route[1:]:
                    distance=math.dist(cell,peer)
                    if (cleared and distance<=clearance) or (not cleared and distance+1e-9<previous):
                        raise RuntimeError('Path enters a held peer corridor')
                    cleared=cleared or distance>clearance;previous=distance
            name=self.names[rid];goal=self.goals[str(tid)][1] if tid is not None else None
            points=self.grid.points(route,snapshot['robots'][name]['pose'],goal)
            for other,state in snapshot['robots'].items():
                if not guard_peers or other==name or self.ids[other] in {v[0] for v in selected}:continue
                p=state['pose']
                for a,b in zip(points,points[1:]):
                    dx,dy=b[0]-a[0],b[1]-a[1];denom=dx*dx+dy*dy
                    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/denom)) if denom else 0.
                    gap=math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)
                    if gap<physical:
                        raise RuntimeError(f'Path violates physical peer clearance: {name} passes {gap:.3f} m '
                                           f'from held {other} (< {physical:.3f} m)')
            limit=self._mission.get('max_action_length_m')
            length=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]))
            if limit is not None and (not finite(limit) or limit<=0 or length>limit):
                raise ValueError('Action exceeds the mission path-length limit')
            actions.append({'robot':name,'task':tid,'points':points,
                            'duration':self.durations.get(tid,0.) if tid is not None else 0.})
        return {'actions':actions,'audit':audit,'intent':intent,'map_sha256':self.grid.signature,'optimality_claim':False}
