"""Low-rate, socket-only mission control. All motor control remains in Nav2/Pi."""
import argparse
import itertools
import json
import math
import os
from pathlib import Path
import select
import socket
import subprocess
import sys
import time
import uuid

from .smoothing import smooth_points


class LocalizationUnavailable(RuntimeError):
    def __init__(self, robot, state):
        self.robot = robot
        self.state = {key: state.get(key) for key in ('localized', 'odom_fresh', 'pose', 'localization')}
        super().__init__(robot+' localization/sensors not ready: '+json.dumps(self.state))

    @property
    def is_stale(self):
        reason = (self.state['localization'] or {}).get('reason')
        return reason in ('stale_sensors', 'stale_scan_timestamp', 'stale_map_tf',
                          'stale_fusion', 'stale_amcl_pose', 'stable_localization')


def request(value,port):
    payload=(json.dumps(value,allow_nan=False)+'\n').encode()
    if len(payload)>65536:raise ValueError('Maximum request size is 65536 bytes including newline')
    with socket.create_connection(('127.0.0.1',port),timeout=2) as connection:
        connection.sendall(payload)
        with connection.makefile('rb') as stream:
            response=stream.readline(16*1024**2+1)
        if not response.endswith(b'\n') or len(response)>16*1024**2:raise RuntimeError('Invalid gateway response')
        result=json.loads(response)
        if not isinstance(result,dict):raise RuntimeError('Expected gateway JSON object')
        if not result.get('ok'):raise RuntimeError('Gateway rejected request: '+json.dumps(result))
        return result


def ready(snapshot):
    if not snapshot.get('robots'):raise RuntimeError('No robot states available')
    for name,state in snapshot['robots'].items():
        if not state.get('localized') or not state.get('odom_fresh') or state.get('pose') is None:
            raise LocalizationUnavailable(name, state)
        if not all(type(v) in (float,int) and math.isfinite(v) for v in state['pose']):
            raise RuntimeError('Nonfinite robot pose')


def wait_for_fleet(port, paths_required, timeout=30.):
    deadline = time.monotonic() + timeout
    stable_at = None
    last_error = 'Fleet is not ready'
    while time.monotonic() < deadline:
        try:
            snapshot = request({'command': 'snapshot'}, port)
            ready(snapshot)
            if any(state['busy'] for state in snapshot['robots'].values()):
                raise RuntimeError('Another robot action is active')
            if paths_required and not all(state['path_ready'] for state in request({'command': 'status'}, port)['robots'].values()):
                raise RuntimeError('Nav2 FollowPath servers are not ready')
            stable_at = stable_at or time.monotonic()
            if time.monotonic() - stable_at >= 1.:
                snapshot = request({'command': 'snapshot', 'include_map': True}, port)
                ready(snapshot)
                if 'map' not in snapshot:
                    raise RuntimeError('No shared map available')
                if any(state['busy'] for state in snapshot['robots'].values()):
                    raise RuntimeError('Another robot action is active')
                return snapshot
        except (OSError, RuntimeError) as error:
            last_error = str(error)
            stable_at = None
        time.sleep(.25)
    raise RuntimeError(last_error)


def wait_recovery(port, map_revision, robots, timeout=12., stable_seconds=1.):
    deadline = time.monotonic() + timeout
    stable_since = None
    while time.monotonic() < deadline:
        snapshot = request({'command': 'snapshot'}, port)
        if snapshot['map_revision'] != map_revision:
            raise RuntimeError('Map changed during recovery; generate a new mission preview')
        states = snapshot.get('robots', {})
        status = request({'command': 'status'}, port).get('robots', {})
        try:
            ready(snapshot)
        except RuntimeError:
            stable_since = None
            time.sleep(.2)
            continue
        stopped = set(states) == set(status) == set(robots)
        for state in states.values():
            velocity = state.get('velocity')
            if (not isinstance(velocity, list) or len(velocity) != 2
                    or not all(type(v) in (int, float) and math.isfinite(v) for v in velocity)
                    or abs(velocity[0]) > .01 or abs(velocity[1]) > .04):
                stopped = False
        if stopped and all(not state['busy'] and state['path_ready'] for state in status.values()):
            now = time.monotonic()
            stable_since = now if stable_since is None else stable_since
            if now - stable_since >= stable_seconds:
                return snapshot
        else:
            stable_since = None
        time.sleep(.2)
    raise TimeoutError('Fleet did not settle with fresh odometry and localization after stop')


class Worker:
    def __init__(self,root,log):
        self.log=Path(log).open('a');self.buffer=b''
        env={**os.environ,'PYTHONDONTWRITEBYTECODE':'1','OPENBLAS_NUM_THREADS':'1','OMP_NUM_THREADS':'1','MKL_NUM_THREADS':'1'}
        self.process=subprocess.Popen([sys.executable,'-B','-m','tb4_marco_bridge.worker','--root',str(root)],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=self.log,env=env,bufsize=0)
        try:
            if self.read(8).get('ready') is not True:raise RuntimeError('Invalid MARCO worker startup')
        except BaseException:
            self.close();raise

    def read(self,timeout):
        end=time.monotonic()+timeout
        while b'\n' not in self.buffer:
            if not select.select([self.process.stdout],[],[],max(0,end-time.monotonic()))[0]:
                raise TimeoutError('MARCO planning deadline exceeded; no motion was dispatched')
            chunk=os.read(self.process.stdout.fileno(),65536)
            if not chunk:raise RuntimeError('MARCO worker exited; inspect worker.log')
            self.buffer+=chunk
            if len(self.buffer)>16*1024**2:raise RuntimeError('Planner response too large')
        line,self.buffer=self.buffer.split(b'\n',1)
        return json.loads(line)

    def call(self,value):
        self.process.stdin.write((json.dumps(value,allow_nan=False)+'\n').encode())
        result=self.read(6)
        if not result.get('ok'):raise RuntimeError('MARCO: '+result.get('error','unknown failure'))
        return result['result']

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
        self.process.stdin.close();self.process.stdout.close();self.log.close()


def tracking_error(position,points):
    best=float('inf')
    for a,b in zip(points,points[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1];denom=dx*dx+dy*dy
        t=max(0.,min(1.,((position[0]-a[0])*dx+(position[1]-a[1])*dy)/denom)) if denom else 0.
        best=min(best,math.hypot(position[0]-a[0]-t*dx,position[1]-a[1]-t*dy))
    return best


class ReplanRequested(RuntimeError):
    pass


class CorridorViolation(RuntimeError):
    pass


def execute_batch(actions,mission,port,map_revision,report,interrupted=None,extra=None):
    owner=uuid.uuid4().hex;active={};completed=[];stale_since=None
    tolerance=mission.get('tracking_tolerance',.04)
    started=time.monotonic();timeout=mission.get('action_timeout_s',180.)+sum(float(a.get('duration') or 0.) for a in actions)
    if type(timeout) not in (int,float) or not math.isfinite(timeout) or not 0<timeout<=600:raise ValueError('action_timeout_s must be 0..600')
    try:
        for action in actions:
            if interrupted and interrupted():raise ReplanRequested('New RViz goal or coordinator shutdown')
            response=request({'command':'path','robot':action['robot'],'points':action['points'],'owner':owner},port)
            active[action['robot']]=action
            report.append({'event':'dispatch','action':action,'response':response})
        while active:
            if interrupted and interrupted():raise ReplanRequested('New RViz goal or coordinator shutdown')
            if time.monotonic()-started>timeout:raise TimeoutError('Execution deadline exceeded')
            snapshot=request({'command':'snapshot'},port)
            if snapshot['map_revision']!=map_revision:raise RuntimeError('Map changed during execution; replan required')
            try:
                ready(snapshot);stale_since=None
            except LocalizationUnavailable as error:
                # One Wi-Fi-delayed scan (stamp > 0.5 s old) must not abort a mission:
                # only a stale state lasting over 1 s counts as lost localization.
                if error.is_stale and time.monotonic()-(stale_since or time.monotonic())<1.:
                    stale_since=stale_since or time.monotonic();time.sleep(.2);continue
                report.append({'event': 'localization_unavailable', 'robot': error.robot, 'state': error.state})
                raise
            status=request({'command':'status'},port)['robots']
            if extra:
                # Concurrent execution: robots released while this batch runs are
                # dispatched at once under the same lease owner.
                for action in extra(dict(active),snapshot,status) or ():
                    if action['robot'] in active:continue
                    response=request({'command':'path','robot':action['robot'],'points':action['points'],'owner':owner},port)
                    active[action['robot']]=action;actions.append(action)
                    timeout+=float(action.get('duration') or 0.)+mission.get('action_timeout_s',180.)/2
                    report.append({'event':'release','action':action,'response':response})
                status=request({'command':'status'},port)['robots']
            for robot,action in list(active.items()):
                state=status[robot];position=snapshot['robots'][robot]['pose']
                distance = tracking_error(position, action.get('reserved', action['points']))
                if distance > tolerance:
                    report.append({'event': 'corridor_violation', 'robot': robot, 'task': action['task'],
                                   'pose': position, 'distance_m': distance, 'tolerance_m': tolerance,
                                   'velocity': snapshot['robots'][robot].get('velocity')})
                    raise CorridorViolation(
                        f'{robot} left reserved path corridor ({distance:.3f} m > {tolerance:.3f} m)')
                if state['busy']:
                    try:
                        request({'command':'lease','robot':robot,'owner':owner},port)
                    except RuntimeError:
                        # Completion can clear ownership between status and lease.
                        state=request({'command':'status'},port)['robots'][robot]
                        if state['busy'] or state['last_result']!='succeeded':raise
                if not state['busy']:
                    if state['last_result']!='succeeded':raise RuntimeError(robot+' path result: '+str(state['last_result']))
                    if math.dist(position[:2],action['points'][-1][:2])>.10:raise RuntimeError(robot+' final position mismatch')
                    # MACRO task action (pick/place/toggle...): the robot holds its
                    # pose for the task duration before the DAG sees it completed.
                    duration=float(action.get('duration') or 0.)
                    if action['task'] is not None and duration>0:
                        if 'acting_since' not in action:
                            action['acting_since']=time.monotonic()
                            report.append({'event':'acting','robot':robot,'task':action['task'],'duration_s':duration})
                        if time.monotonic()-action['acting_since']<duration:continue
                    if action['task'] is not None:completed.append(action['task'])
                    report.append({'event':'completed','robot':robot,'task':action['task'],'pose':position})
                    request({'command':'clear_preview','robot':robot},port)
                    del active[robot]
            time.sleep(.2)
        time.sleep(.6)  # Let the permission/stamper/guard stop chain settle at the batch barrier.
        return completed
    finally:
        # Cancellation is supplemented by the gateway lease and command permission
        # expiry; a killed Python process cannot leave an unleased path executing.
        for robot in active:
            try:request({'command':'cancel','robot':robot},port)
            except Exception:pass


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command',choices=['status','plan','run'])
    parser.add_argument('mission',nargs='?',type=Path)
    parser.add_argument('--root',type=Path,default=Path(os.environ.get('MARCO_ROOT','MACRO')))
    parser.add_argument('--port',type=int,default=int(os.environ.get('TB4_SOCKET_PORT','8765')))
    parser.add_argument('--out',type=Path,default=Path('log/marco/mission.json'))
    parser.add_argument('--execute',action='store_true',help='Explicitly permit physical path execution')
    parser.add_argument('--full',action='store_true',help='Check every batch offline before showing the first preview')
    args=parser.parse_args()
    if args.full and args.command!='plan':parser.error('--full requires plan')
    if args.command=='status':print(json.dumps(request({'command':'snapshot'},args.port),indent=2));return
    if args.command=='run' and not args.execute:parser.error('run requires --execute; plan never moves robots')
    if args.mission is None:parser.error('mission JSON is required')
    args.out.parent.mkdir(parents=True,exist_ok=True)
    completed = []
    report={'result':'incomplete','events':[],'completed_tasks':completed,
            'execution_requested':args.command=='run','marco_root':str(args.root.resolve())}
    worker=None
    try:
        mission=json.loads(args.mission.read_text())
        deviation = mission.get('smooth_deviation_m', 0.)
        if (type(deviation) not in (int, float) or not math.isfinite(deviation)
                or not 0 <= deviation <= mission.get('tracking_tolerance', .04) / 2):
            raise ValueError('smooth_deviation_m must be within 0..tracking_tolerance/2')
        snapshot = wait_for_fleet(args.port, paths_required=args.command == 'run')
        worker=Worker(args.root.resolve(),args.out.parent/'worker.log')
        assignment=worker.call({'command':'assign','snapshot':snapshot,'mission':mission});report['assignment']=assignment
        base_map=snapshot['map'];map_revision=snapshot['map_revision'];intent=None
        fleet = tuple(snapshot['robots'])
        recovered = False
        recovery_snapshot = None
        for batch_index in range(32):
            if args.command=='run':
                snapshot = recovery_snapshot or request({'command':'snapshot'},args.port)
                recovery_snapshot = None
                ready(snapshot)
                if snapshot['map_revision']!=map_revision:raise RuntimeError('Map changed; generate a new mission preview')
                snapshot['map']=base_map
            batch=worker.call({'command':'next','snapshot':snapshot,'mission':mission,'assignment':assignment,'completed':completed,'intent':intent})
            intent=batch.get('intent')
            if deviation:
                for action in batch['actions']:
                    action['reserved'] = action['points']
                    action['points'] = smooth_points(action['points'], deviation)
            report['events'].append({'event':'plan','batch':batch})
            if args.full:
                if not batch['actions']:
                    goals={task['id']:task['goal'] for task in mission['tasks']}
                    if batch.get('reason')!='completed' or sorted(completed)!=sorted(goals):
                        raise RuntimeError('DAG cannot make progress')
                    for robot,tasks in assignment['sequences'].items():
                        if tasks and math.dist(snapshot['robots'][robot]['pose'][:2],goals[tasks[-1]][:2])>.025:
                            raise RuntimeError(robot+' would be relocated away from its final goal')
                    for action in report['events'][0]['batch']['actions']:
                        request({'command':'preview','robot':action['robot'],'points':action['points'],
                                 'task':action['task']},args.port)
                    report['result']='full_preview'
                    break
                for action in batch['actions']:
                    snapshot['robots'][action['robot']]['pose']=action['points'][-1]
                    if action['task'] is not None:completed.append(action['task'])
                continue
            for action in batch['actions']:
                request({'command':'preview','robot':action['robot'],'points':action['points'],
                         'task':action['task']},args.port)
            if args.command=='plan':report['result']='preview';break
            if not batch['actions']:
                if batch.get('reason')!='completed':raise RuntimeError('DAG cannot make progress')
                report['result']='succeeded';break
            try:
                completed+=execute_batch(batch['actions'],mission,args.port,map_revision,report['events'])
            except (CorridorViolation, LocalizationUnavailable) as error:
                # A single relocation cannot have completed a numbered task.
                if recovered or len(batch['actions']) != 1 or batch['actions'][0]['task'] is not None:
                    raise
                if isinstance(error, LocalizationUnavailable) and not error.is_stale:
                    raise
                recovered = True
                print(f'{error}. Waiting for the fleet to stop before replanning.', file=sys.stderr, flush=True)
                recovery_snapshot = wait_recovery(args.port, map_revision, fleet)
                event = 'corridor_replan' if isinstance(error, CorridorViolation) else 'localization_replan'
                report['events'].append({'event': event,
                                         'poses': {name: state['pose'] for name, state in recovery_snapshot['robots'].items()}})
                intent = None
        else:raise RuntimeError('Bounded intervention limit exceeded')
        print(json.dumps(report,indent=2),flush=True)
    except BaseException as error:
        report.update(result='failed',error=type(error).__name__+': '+str(error))
        raise
    finally:
        if worker:worker.close()
        args.out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
