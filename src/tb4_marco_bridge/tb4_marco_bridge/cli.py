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


def request(value,port):
    with socket.create_connection(('127.0.0.1',port),timeout=2) as connection:
        connection.sendall((json.dumps(value,allow_nan=False)+'\n').encode())
        response=connection.makefile('rb').readline(16*1024**2)
        if not response or len(response)>=16*1024**2:raise RuntimeError('Invalid gateway response')
        result=json.loads(response)
        if not result.get('ok'):raise RuntimeError('Gateway rejected request: '+json.dumps(result))
        return result


def ready(snapshot):
    if not snapshot.get('robots'):raise RuntimeError('No robot states available')
    for name,state in snapshot['robots'].items():
        if not state.get('localized') or not state.get('odom_fresh') or state.get('pose') is None:
            raise RuntimeError(name+' localization/sensors not ready: '+json.dumps(state.get('localization',{})))
        if not all(type(v) in (float,int) and math.isfinite(v) for v in state['pose']):
            raise RuntimeError('Nonfinite robot pose')


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


def execute_batch(actions,mission,port,map_revision,report):
    owner=uuid.uuid4().hex;active={};completed=[]
    tolerance=mission.get('tracking_tolerance',.04)
    started=time.monotonic();timeout=mission.get('action_timeout_s',180.)
    if type(timeout) not in (int,float) or not math.isfinite(timeout) or not 0<timeout<=600:raise ValueError('action_timeout_s must be 0..600')
    try:
        for action in actions:
            response=request({'command':'path','robot':action['robot'],'points':action['points'],'owner':owner},port)
            active[action['robot']]=action
            report.append({'event':'dispatch','action':action,'response':response})
        while active:
            if time.monotonic()-started>timeout:raise TimeoutError('Execution deadline exceeded')
            snapshot=request({'command':'snapshot'},port);ready(snapshot)
            if snapshot['map_revision']!=map_revision:raise RuntimeError('Map changed during execution; replan required')
            status=request({'command':'status'},port)['robots']
            for robot,action in list(active.items()):
                state=status[robot];position=snapshot['robots'][robot]['pose']
                if tracking_error(position,action['points'])>tolerance:raise RuntimeError(robot+' left reserved path corridor')
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
    parser.add_argument('--root',type=Path,default=Path(os.environ.get('MARCO_ROOT','MARCO')))
    parser.add_argument('--port',type=int,default=int(os.environ.get('TB4_SOCKET_PORT','8765')))
    parser.add_argument('--out',type=Path,default=Path('log/marco/mission.json'))
    parser.add_argument('--execute',action='store_true',help='Explicitly permit physical path execution')
    args=parser.parse_args()
    if args.command=='status':print(json.dumps(request({'command':'snapshot'},args.port),indent=2));return
    if args.command=='run' and not args.execute:parser.error('run requires --execute; plan never moves robots')
    if args.mission is None:parser.error('mission JSON is required')
    args.out.parent.mkdir(parents=True,exist_ok=True)
    report={'result':'incomplete','events':[],'execution_requested':args.command=='run','marco_root':str(args.root.resolve())}
    worker=None
    try:
        mission=json.loads(args.mission.read_text())
        snapshot=request({'command':'snapshot','include_map':True},args.port);ready(snapshot)
        if any(s['busy'] for s in snapshot['robots'].values()):raise RuntimeError('Another robot action is active')
        if 'map' not in snapshot:raise RuntimeError('No shared map available')
        if args.command=='run' and not all(s['path_ready'] for s in request({'command':'status'},args.port)['robots'].values()):
            raise RuntimeError('Nav2 path servers not ready; localization-only mode permits preview, not execution')
        worker=Worker(args.root.resolve(),args.out.parent/'worker.log')
        assignment=worker.call({'command':'assign','snapshot':snapshot,'mission':mission});report['assignment']=assignment
        base_map=snapshot['map'];map_revision=snapshot['map_revision'];completed=[];intent=None
        for batch_index in range(32):
            if args.command=='run':
                snapshot=request({'command':'snapshot'},args.port);ready(snapshot)
                if snapshot['map_revision']!=map_revision:raise RuntimeError('Map changed; generate a new mission preview')
                snapshot['map']=base_map
            batch=worker.call({'command':'next','snapshot':snapshot,'mission':mission,'assignment':assignment,'completed':completed,'intent':intent})
            intent=batch.get('intent')
            report['events'].append({'event':'plan','batch':batch})
            for action in batch['actions']:request({'command':'preview','robot':action['robot'],'points':action['points']},args.port)
            if args.command=='plan':report['result']='preview';break
            if not batch['actions']:
                if batch.get('reason')!='completed':raise RuntimeError('DAG cannot make progress')
                report['result']='succeeded';break
            completed+=execute_batch(batch['actions'],mission,args.port,map_revision,report['events'])
        else:raise RuntimeError('Bounded intervention limit exceeded')
        report['completed_tasks']=completed
        print(json.dumps(report,indent=2),flush=True)
    except BaseException as error:
        report.update(result='failed',error=type(error).__name__+': '+str(error))
        raise
    finally:
        if worker:worker.close()
        args.out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')


if __name__=='__main__':main()
