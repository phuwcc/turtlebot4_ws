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
        # The planning disc (coordination radius) is wider than the body: a robot
        # that drifted a few cm towards a wall (corner cutting, AMCL jitter) is
        # still physically clear, so plan from the nearest footprint-safe cell.
        snap=float(mission.get('snap_to_safe_m',.08))/self.grid.res
        reach=[(dx,dy) for dx in range(-math.ceil(snap),math.ceil(snap)+1) for dy in range(-math.ceil(snap),math.ceil(snap)+1)
               if math.hypot(dx,dy)<=snap]
        reach.sort(key=lambda d:math.hypot(*d))
        for rid,c in list(self.positions.items()):
            if c in self.grid.cells:continue
            near=next(((c[0]+dx,c[1]+dy) for dx,dy in reach if (c[0]+dx,c[1]+dy) in self.grid.cells),None)
            if near is None:
                raise ValueError('A robot footprint overlaps unknown/occupied map cells; check localization')
            self.positions[rid]=near
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
        staged={}
        def drive_route(rid,goal):
            """Driving is the lower layer: A* from the robot's cell to the goal that
            MARCO (or the task) chose, around the bodies of every other robot
            (pipelined staging: earlier movers at their holding goals)."""
            centres=[staged[o] if o in staged else (self.positions[o] if self.marco_radius is not None else
                      self.grid.continuous_cell(snapshot['robots'][self.names[o]]['pose']))
                     for o in self.positions if o!=rid]
            return self.algorithms._astar_around_robot_bodies(self.grid.cells,staged.get(rid,self.positions[rid]),goal,
                robot_centres=centres,clearance_cells=clearance)
        if intent:
            priority=intent['priority'];tid=intent['task']
            if tid in completed:
                # The priority robot was released early (concurrent execution) and
                # already finished: the intervention is over; continue nominally.
                return self.next_batch(snapshot,mission,assignment,sorted(completed),None)
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
            held_priority=False
            if (self._mission.get('hold_priority',True) and
                    any(item['robot_id']==plan['priority_robot'] for item in plan.get('relocation_sequence',[]))):
                # User rule: the priority robot MARCO picked waits in place until the
                # others have cleared its route; it does not step aside itself. Re-run
                # MARCO's relocation search for that same priority robot (the search
                # never moves the priority robot); keep the original plan only if no
                # such clearance exists.
                keep=plan['priority_robot']
                def moves(candidate):
                    steps=candidate.get('relocation_sequence') or candidate.get('relocations') or ()
                    return any(step.get('robot_id')==keep for step in steps)
                def priority_waits(leaves,*_):
                    # MARCO still scores the remaining candidates; only those that
                    # move the priority robot are dropped.
                    return [c for c in leaves if not moves(c)] or leaves
                alt=self.coordinator.search_relocation_plan(robots=states,dag_robots=states,tasks=tasks,
                    task_status=task_status,objects=objects,navigation_cells=self.grid.cells,robot_radius=radius,
                    grid_size=self.grid.res,speed_cells_per_second=self.speed/self.grid.res,
                    planning_timeout_s=float(self._mission.get('planning_timeout_s',5.)),
                    local_time_evaluator=timing,selection_policy='dag_aware',
                    priority_robot_ids=[keep],dag_candidate_refiner=priority_waits)
                if alt and alt['priority_robot']==keep and not moves(alt):
                    plan=alt;held_priority=True
            audit={key:plan.get(key) for key in ('strategy','selected_hypothesis','priority_robot','local_resolution_delay','estimated_makespan')}
            audit['priority_held_in_place']=held_priority
            relocations=plan.get('relocation_sequence',[])
            # Use MARCO's decision (priority branch + ordered temporary holding
            # goals); each move is driven by A* to that goal, not MARCO's grid path.
            audit['holding_goals']=[{'robot':self.names[item['robot_id']],
                'goal':list(self.grid.world(tuple(item['path'][-1]))[:2])} for item in relocations]
            if relocations:
                # MACRO ROS executor (three_robot_t/execute.py, pipelined staging):
                # every holding action and the priority resume form one batch; each
                # starts as soon as its route is disjoint from the remaining routes of
                # all earlier unfinished actions (execute_batch gates on 'stage').
                # Routes are A* over the staged poses (earlier movers at their goals).
                # A robot may appear more than once (e.g. the priority robot first
                # steps aside, or a composed clearance moves a robot twice): its later
                # action starts where the earlier one ended, after it finished.
                priority=plan['priority_robot'];selected=[]
                protected=[c for c in plan.get('planned_paths',{}).get(priority,[])]
                for item in relocations:
                    rid=item['robot_id'];goal=self.deepen(tuple(item['path'][-1]),item['path'],rid,staged,protected,clearance)
                    route=drive_route(rid,goal)
                    if route is None:raise RuntimeError(f'No A* route to MARCO holding goal for {self.names[rid]}')
                    selected.append((rid,route,None));staged[rid]=goal
                route=drive_route(priority,self.goals[str(current[priority]['id'])][0])
                if route is None:raise RuntimeError(f'No A* route for priority robot {self.names[priority]}')
                selected.append((priority,route,current[priority]['id']))
                audit['pipelined']=True
                batch=self.package_actions(snapshot,selected,audit,None)
                for index,action in enumerate(batch['actions']):action['stage']=index
                return batch
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

    def release(self, snapshot, mission, assignment, completed, moving):
        """MACRO-style release while a batch runs. ``moving`` maps robot name ->
        remaining metric path of every robot that is currently driving. A robot
        with a ready task that is not moving is released at once when its A*
        route to the task goal keeps the coordination clearance from every moving
        robot's remaining path and from every other body."""
        self.prepare(snapshot,mission)
        assigned={int(k):self.ids[v] for k,v in assignment['assignment'].items()}
        completed=set(completed);task_by_id={t['id']:dict(t,robot=assigned[t['id']]) for t in self.tasks}
        clearance=2*self.dynamic_radius()/self.grid.res
        remaining_cells={self.ids[name]:[self.grid.cell(p) for p in points] for name,points in moving.items() if points}
        selected=[]
        for name,sequence in assignment['sequences'].items():
            rid=self.ids[name]
            if name in moving:continue
            todo=[tid for tid in sequence if tid not in completed]
            if not todo or not set(task_by_id[todo[0]]['node'])<=completed:continue
            task=task_by_id[todo[0]];goal=self.goals[str(task['id'])][0]
            if self.positions[rid]==goal:continue
            route=self.algorithms._astar_around_robot_bodies(self.grid.cells,self.positions[rid],goal,
                robot_centres=[c for o,c in self.positions.items() if o!=rid],clearance_cells=clearance)
            if route is None:continue
            if any(self.conflict(route,cells,clearance) for cells in remaining_cells.values()):continue
            if any(self.conflict(route,other,clearance) for _,other,_ in selected):continue
            selected.append((rid,route,task['id']))
        if not selected:return {'actions':[],'audit':{'strategy':'release'},'intent':None}
        moving_ids={self.ids[n] for n in moving}
        # Moving robots are not held peers; their remaining paths were checked above.
        held=[r for r in self.robots if self.ids[r] not in moving_ids]
        sub=dict(snapshot,robots={r:snapshot['robots'][r] for r in held})
        return self.package_actions(sub,selected,{'strategy':'release','released':[self.names[r] for r,_,_ in selected]},None)

    def dynamic_radius(self):
        if self.marco_radius is not None:
            return self.radius  # identical clearance to MACRO's planner and executor
        # Static inflation already includes cell area. For peer paths account for
        # residual grid quantization without charging the same margin twice.
        # The exact metric segment audit below independently enforces physical clearance.
        return self.radius+max(0.,math.sqrt(.5)*self.grid.res-self._mission.get('safety_margin',.05)+.01)

    def deepen(self,goal,path,rid,staged,protected,clearance):
        """Park a yielding robot deeper than MARCO's minimal holding goal.

        MARCO's holding goal only just clears the priority corridor; for the
        real robots (and to let the priority robot start while the yielder is
        still entering) continue along the yielder's own approach direction by
        up to ``holding_extra_m`` while every cell stays footprint-safe, keeps
        the clearance from the other robots (at their staged/current cells) and
        moves away from the priority robot's protected path.
        """
        extra=float(self._mission.get('holding_extra_m',0.))/self.grid.res  # off: MARCO's own holding goals
        if extra<=0 or len(path)<2:return goal
        a=path[max(0,len(path)-4)];dx,dy=goal[0]-a[0],goal[1]-a[1];n=math.hypot(dx,dy)
        if n<1e-6:return goal
        dx,dy=dx/n,dy/n
        others=[staged.get(o,self.positions[o]) for o in self.positions if o!=rid]
        away=lambda c:min((math.dist(c,q) for q in protected),default=float('inf'))
        best=goal
        for k in range(1,int(extra)+1):
            cell=(round(goal[0]+dx*k),round(goal[1]+dy*k))
            if cell not in self.grid.cells:break
            if any(math.dist(cell,o)<=clearance for o in others):break
            if away(cell)+1e-9<away(best):break
            best=cell
        return best

    def straighten(self,route,centres,clearance):
        """Any-angle shortcut of an 8-connected A* route (string pulling).

        A grid route only moves in 45-degree steps, so a goal a few cm to the side
        starts with a diagonal kink and the controller turns in place first. Each
        shortcut keeps every sample inside the footprint-safe cells (all cells
        around it) and outside the clearance disc of every other robot body (a
        start inside a disc may only move outwards, as in MACRO's A*). Output is
        densified to <= 1 cell spacing for the gateway.
        """
        if len(route)<3:return route
        # A robot parked close to a wall starts outside the footprint-safe set;
        # A* still leaves from there, so its own route cells count as free.
        cells=self.grid.cells|{(round(c[0]),round(c[1])) for c in route}
        def samples(a,b):
            n=max(1,math.ceil(math.dist(a,b)/.25))
            return [(a[0]+(b[0]-a[0])*k/n,a[1]+(b[1]-a[1])*k/n) for k in range(n+1)]
        def free(a,b):
            previous={i:math.dist(a,c) for i,c in enumerate(centres)}
            for p in samples(a,b)[1:]:
                x,y=p
                if (round(x),round(y)) not in cells:return False
                for i,c in enumerate(centres):
                    d=math.dist(p,c)
                    if d<=clearance and d+1e-9<previous[i]:return False
                    previous[i]=d
            return True
        result=[route[0]];i=0;last=len(route)-1
        while i<last:
            j=last
            while j>i+1 and not free(route[i],route[j]):j-=1
            a,b=route[i],route[j];n=max(1,math.ceil(math.dist(a,b)))
            result+=[(a[0]+(b[0]-a[0])*k/n,a[1]+(b[1]-a[1])*k/n) for k in range(1,n+1)]
            i=j
        return result

    def package_actions(self,snapshot,selected,audit,intent,guard_peers=True):
        actions=[]
        clearance=2*self.dynamic_radius()/self.grid.res
        # MARCO verifies clearance between grid-cell centres; a measured pose sits
        # up to half a cell diagonal from its centre. Bodies (0.35 m) still keep
        # >= 5 cm apart at this threshold.
        physical=(2*self.radius-math.sqrt(.5)*self.grid.res if self.marco_radius is not None else
                  2*(self._mission.get('robot_radius',.175)+self._mission.get('tracking_tolerance',.04))+.02)
        moving={item[0] for item in selected}
        straightened=[];seen={}
        for rid,route,tid in selected:
            # Straight segments instead of 45-degree grid steps: fewer turns in place.
            # Bodies: parked robots now, movers at the end of their routes.
            centres=[self.positions[o] for o in self.positions if o!=rid and o not in moving]
            if guard_peers:
                # Intervention batch: keep clear of where the other movers end up.
                # (Nominal batches are guarded by the swept-contact trigger instead.)
                centres+=[r[-1] for o,r,_ in selected if o!=rid]
            # Shortcut from the measured pose to the exact task goal, so the first
            # and last segments do not kink towards cell centres.
            ends=list(route)
            if len(ends)>=2:
                # A robot with a second staged action starts where its first ended.
                ends[0]=seen[rid] if rid in seen else self.grid.continuous_cell(snapshot['robots'][self.names[rid]]['pose'])
                if tid is not None:ends[-1]=self.grid.continuous_cell(self.goals[str(tid)][1])
            path=self.straighten(ends,centres,clearance)
            straightened.append((rid,path if path is not ends else route,tid));seen[rid]=straightened[-1][1][-1]
        selected=straightened;last_pose={}
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
            pose=last_pose.get(name,snapshot['robots'][name]['pose'])
            arrive=float(self._mission.get('arrive_tolerance_m',.07))
            if goal is not None and math.dist(pose[:2],goal[:2])<=arrive:
                # Already at the task location (e.g. stopped a few cm past it by the
                # contact trigger): do the task here instead of a few-cm correction
                # that would make a differential-drive robot turn around twice.
                final=goal[2] if len(goal)==3 else pose[2]
                actions.append({'robot':name,'task':tid,'points':[list(pose[:3]),[pose[0],pose[1],final]],
                                'duration':self.durations.get(tid,0.)})
                last_pose[name]=actions[-1]['points'][-1]
                continue
            points=self.grid.points(route,pose,goal)
            # Holding move of a yielding/idle robot: its heading does not matter, so
            # it never turns in place at the holding goal (yaw-free goal checker).
            # A task goal given without a yaw (object pose [x, y]) is heading-free too.
            extra={'goal_checker':'holding_goal_checker'} if tid is None or len(goal)==2 else {}
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
                            'duration':self.durations.get(tid,0.) if tid is not None else 0.,**extra})
            last_pose[name]=points[-1]
        return {'actions':actions,'audit':audit,'intent':intent,'map_sha256':self.grid.signature,'optimality_claim':False}
