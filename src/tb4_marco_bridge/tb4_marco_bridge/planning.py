"""Physical-robot adapter to MARCO's unmodified, simulator-free core."""
import hashlib
import heapq
import importlib
import json
import math
import sys
from pathlib import Path


def finite(value):
    return type(value) in (float, int) and math.isfinite(value)


def load_core(root):
    root = Path(root).expanduser().resolve()
    if not (root/'src/core/coordination/coordinator.py').is_file():
        raise ValueError('MARCO_ROOT must point to the original MARCO checkout')
    # No __pycache__, build metadata, scenario outputs or logs inside MARCO.
    sys.dont_write_bytecode = True
    sys.path.insert(0, str(root/'src'))
    return importlib.import_module('core')


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

    def cell(self, pose):
        x,y = pose[0]-self.origin[0], pose[1]-self.origin[1]
        return math.floor((self.cos*x+self.sin*y)/self.res), math.floor((-self.sin*x+self.cos*y)/self.res)

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
        from core.coordination import planner, coordinator, relocation_search
        from core.models import RobotState
        self.algorithms, self.coordinator = planner, coordinator
        self.conflict=relocation_search._path_conflict
        self.RobotState=RobotState
        self.grid=None

    def prepare(self, snapshot, mission):
        if not isinstance(mission,dict) or not isinstance(mission.get('tasks'),list) or not 0 < len(mission['tasks']) <= 8:
            raise ValueError('Mission needs 1..8 navigation tasks')
        self.radius=mission.get('robot_radius',.175)+mission.get('safety_margin',.05)+mission.get('tracking_tolerance',.04)
        self.speed=mission.get('speed_mps',.05)
        if not finite(self.radius) or not .175 <= self.radius <= .6 or not finite(self.speed) or not 0 < self.speed <= .05:
            raise ValueError('Invalid robot clearance or validation speed (maximum 0.05m/s)')
        if not self.grid or self.grid.message != snapshot['map'] or self.last_radius != self.radius:
            self.grid=Grid(snapshot['map'],self.radius);self.last_radius=self.radius
        self.robots=tuple(sorted(snapshot['robots']))
        if not self.robots or any(r not in ('robot1','robot2','robot3') for r in self.robots):
            raise ValueError('Unsupported robot identifiers')
        self.ids={r:'R'+r[-1] for r in self.robots};self.names={v:k for k,v in self.ids.items()}
        self.positions={self.ids[r]:self.grid.cell(snapshot['robots'][r]['pose']) for r in self.robots}
        if any(c not in self.grid.cells for c in self.positions.values()):
            raise ValueError('A robot footprint overlaps unknown/occupied map cells; check localization')
        self.goals={};self.tasks=[]
        for t in mission['tasks']:
            if not isinstance(t,dict) or type(t.get('id')) is not int or t['id']<=0:
                raise ValueError('Each task needs a unique positive integer id')
            goal=t.get('goal');candidates=t.get('candidates',list(self.robots));dependencies=t.get('depends_on',[])
            if not isinstance(goal,list) or len(goal) not in (2,3) or not all(finite(v) for v in goal):
                raise ValueError('Task goal must be finite [x,y] or [x,y,yaw] in map')
            if not isinstance(candidates,list) or not candidates or any(r not in self.ids for r in candidates):
                raise ValueError('Candidates must reference active robots')
            if not isinstance(dependencies,list) or any(type(v) is not int for v in dependencies):
                raise ValueError('depends_on must contain task IDs')
            cell=self.grid.cell(goal)
            if cell not in self.grid.cells:raise ValueError('Goal is not footprint-safe in the map')
            self.goals[str(t['id'])]=(cell,goal)
            self.tasks.append({'id':t['id'],'candidate':[self.ids[r] for r in candidates],
                               'node':dependencies,'action':'navigate','object':str(t['id']),'duration':.001})
        self.core.validate_tasks(self.tasks)
        self._mission=mission

    def assign(self, snapshot, mission):
        self.prepare(snapshot,mission)
        parent=self
        class Distance:
            def plan(self,robot,location,start_position=None):
                rid=parent.ids[robot]
                start=start_position if start_position is not None else parent.positions[rid]
                goal=parent.goals[location][0]
                route=parent.algorithms.astar(parent.grid.cells,start,goal)
                if route is None:raise ValueError('No static route for task '+location)
                return parent.algorithms.path_length(route,parent.grid.res),goal
        mode='chain_bnb'
        try:
            from core.assignment_cpp import require_native
            require_native();mode='chain_bnb_cpp'
        except (ImportError,RuntimeError):pass
        result=self.core.optimize_task_assignments(self.tasks,Distance(),robot_speed=self.speed,service_time=.001,search_mode=mode)
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
            task=current.get(rid);goal=self.goals[task['object']][0] if task else None
            route=self.algorithms.astar(self.grid.cells,start,goal) if task else [start]
            if route is None:raise ValueError('No route from current robot position')
            routes[rid]=route
            states.append(self.RobotState(rid,start,(0,0,0),goal=goal,path=route,phase='preview' if task else 'idle',current_task=task))
        clearance=2*radius/self.grid.res
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
            route=self.algorithms._astar_around_robot_bodies(self.grid.cells,self.positions[rid],goal,
                robot_centres=[p for r,p in self.positions.items() if r!=rid],clearance_cells=clearance)
            if route is None:raise RuntimeError('Committed clearance/priority path is no longer feasible')
            return self.package_actions(snapshot,[(rid,route,task_id)],{'strategy':'committed_clearance' if remaining else 'committed_priority'},next_intent)
        # Protect complete routes, including stationary peer footprints, rather
        # than assuming a simulator's predicted arrival times are exact.
        conflicts=any(self.conflict(routes[a],routes[b],clearance) for i,a in enumerate(routes) for b in list(routes)[i+1:])
        audit={'strategy':'disjoint_routes'}
        selected=[(r,routes[r],current[r]['id']) for r in current]
        next_intent=None
        if conflicts:
            objects={name:{'cell':cell,'cells':{cell},'interaction_cells':{cell},'blocking':False} for name,(cell,_) in self.goals.items()}
            def timing(candidate,*_):
                # Conservative metric timing supplied by this physical adapter.
                steps=candidate.get('relocation_sequence',[])
                distance=sum(self.algorithms.path_length(s['path'],self.grid.res) for s in steps)
                priority=candidate['priority_robot']
                route=candidate['preferred_paths'].get(priority,[])
                return distance/self.speed+self.algorithms.path_length(route,self.grid.res)/self.speed+.5*(len(steps)+1)
            plan=self.coordinator.compute_conflict_plan(cluster=states,dag_robots=states,tasks=tasks,
                task_status={t['id']:'completed' if t['id'] in completed else 'pending' for t in tasks},objects=objects,
                navigation_cells=self.grid.cells,robot_radius=radius,grid_size=self.grid.res,
                speed_cells_per_second=self.speed/self.grid.res,planning_timeout_s=3.,hypothesis_workers=1,
                local_resolution_time_evaluator=timing,selection_policy='dag_aware',release_prediction_model='dynamic_events_guarded')
            if not plan:raise RuntimeError('No feasible guarded MARCO intervention')
            audit={key:plan.get(key) for key in ('strategy','selected_hypothesis','priority_robot','local_resolution_delay','estimated_makespan')}
            relocations=plan.get('relocation_sequence',[])
            if relocations:
                step=relocations[0];selected=[(step['robot_id'],step['path'],None)]
                priority=plan['priority_robot']
                next_intent={'priority':priority,'task':current[priority]['id'],
                    'relocations':[{'robot':item['robot_id'],'destination':list(item['path'][-1])} for item in relocations[1:]]}
            else:
                rid=plan['priority_robot'];selected=[(rid,plan['preferred_paths'][rid],current[rid]['id'])]
        return self.package_actions(snapshot,selected,audit,next_intent)

    def dynamic_radius(self):
        # Static inflation already includes cell area. For peer paths account for
        # residual grid quantization without charging the same margin twice.
        # The exact metric segment audit below independently enforces physical clearance.
        return self.radius+max(0.,math.sqrt(.5)*self.grid.res-self._mission.get('safety_margin',.05)+.01)

    def package_actions(self,snapshot,selected,audit,intent):
        actions=[]
        clearance=2*self.dynamic_radius()/self.grid.res
        physical=2*(self._mission.get('robot_radius',.175)+self._mission.get('tracking_tolerance',.04))+.02
        for rid,route,tid in selected:
            peers=[p for r,p in self.positions.items() if r!=rid and r not in {s[0] for s in selected}]
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
                if other==name or self.ids[other] in {v[0] for v in selected}:continue
                p=state['pose']
                for a,b in zip(points,points[1:]):
                    dx,dy=b[0]-a[0],b[1]-a[1];denom=dx*dx+dy*dy
                    t=max(0.,min(1.,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/denom)) if denom else 0.
                    if math.hypot(p[0]-a[0]-t*dx,p[1]-a[1]-t*dy)<physical:
                        raise RuntimeError('Path violates physical peer clearance')
            limit=self._mission.get('max_action_length_m')
            length=sum(math.dist(a[:2],b[:2]) for a,b in zip(points,points[1:]))
            if limit is not None and (not finite(limit) or limit<=0 or length>limit):
                raise ValueError('Action exceeds the mission path-length limit')
            actions.append({'robot':name,'task':tid,'points':points})
        return {'actions':actions,'audit':audit,'intent':intent,'map_sha256':self.grid.signature,'optimality_claim':False}
