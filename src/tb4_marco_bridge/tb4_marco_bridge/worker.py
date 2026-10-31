"""Bounded stdio planning worker. No ROS publishers or robot connections."""
import argparse
import json
import os
import resource
import sys
import traceback
from pathlib import Path
from .planning import Planner


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,required=True);args=parser.parse_args()
    os.nice(10)
    resource.setrlimit(resource.RLIMIT_AS,(2*1024**3,2*1024**3))
    planner=Planner(args.root)
    print(json.dumps({'ready':True}),flush=True)
    for line in sys.stdin:
        try:
            if len(line)>16*1024**2:raise ValueError('Request too large')
            request=json.loads(line)
            if request.get('shutdown'):break
            if request['command']=='assign':result=planner.assign(request['snapshot'],request['mission'])
            elif request['command']=='next':result=planner.next_batch(request['snapshot'],request['mission'],request['assignment'],request['completed'],request.get('intent'))
            else:raise ValueError('Unknown planner request')
            print(json.dumps({'ok':True,'result':result},allow_nan=False),flush=True)
        except Exception as error:
            traceback.print_exc(file=sys.stderr)
            print(json.dumps({'ok':False,'error':str(error)}),flush=True)


if __name__=='__main__':main()
