#!/usr/bin/env python3
"""Real MARCO core, synthetic maps. No ROS nodes or physical robot commands."""
import sys
sys.dont_write_bytecode=True
import json
import math
from pathlib import Path
import subprocess
import unittest
from tb4_marco_bridge.planning import Planner,Grid
from tb4_marco_bridge.cli import Worker,tracking_error,execute_batch
from unittest.mock import patch

ROOT=Path(__file__).resolve().parents[1]

def room():
    return {'width':40,'height':40,'resolution':.1,'origin':[-2,-2,0],'data':[0]*1600}

def snapshot():
    return {'map':room(),'robots':{'robot1':{'pose':[-1,0,0]},'robot2':{'pose':[-1,1,0]}}}

class MarcoPlanningTest(unittest.TestCase):
    def setUp(self):self.planner=Planner(ROOT/'MARCO')

    def test_rotated_map_and_unknown_clearance(self):
        m=room();m['origin']=[2,-3,math.pi/3];m['data'][20*40+20]=-1
        grid=Grid(m,.2)
        self.assertEqual(grid.cell(grid.world((10,12))),(10,12))
        self.assertNotIn((20,20),grid.cells)
        self.assertNotIn((21,20),grid.cells)
        self.assertNotIn((0,15),grid.cells)

    def test_assignment_respects_future_endpoints_and_dependencies(self):
        s=snapshot();m={'tasks':[{'id':1,'goal':[0,0,0],'candidates':['robot1']},
                                {'id':2,'goal':[1,0,0],'candidates':['robot1'],'depends_on':[1]}]}
        a=self.planner.assign(s,m)
        self.assertEqual(a['sequences']['robot1'],[1,2]);self.assertAlmostEqual(a['total_distance_m'],2.,places=5)
        batch=self.planner.next_batch(s,m,a,[])
        self.assertEqual([x['task'] for x in batch['actions']],[1])
        s['robots']['robot1']['pose']=[0,0,0]
        self.assertEqual(self.planner.next_batch(s,m,a,[1])['actions'][0]['task'],2)
        self.assertEqual(self.planner.next_batch(s,m,a,[1,2])['reason'],'completed')

    def test_parallel_routes_and_bad_missions(self):
        s=snapshot();m={'tasks':[{'id':1,'goal':[1,0,0],'candidates':['robot1']},
                                {'id':2,'goal':[1,1,0],'candidates':['robot2']}]}
        a=self.planner.assign(s,m);batch=self.planner.next_batch(s,m,a,[])
        self.assertEqual(len(batch['actions']),2)
        self.assertTrue(all(len(v['points'])>2 for v in batch['actions']))
        self.assertEqual(tracking_error([.5,.1],[[0,0,0],[1,0,0]]),.1)
        for goal in ([float('nan'),0],[10,0],[True,0]):
            with self.subTest(goal=goal),self.assertRaises(ValueError):
                self.planner.assign(s,{'tasks':[{'id':1,'goal':goal}]})
        with self.assertRaises(ValueError):self.planner.assign(s,{'tasks':[{'id':1,'goal':[0,0],'depends_on':[1]}]})

    def test_actual_dag_conflict_coordinator(self):
        s=snapshot();s['robots']['robot2']['pose']=[1,0,math.pi]
        m={'tasks':[{'id':1,'goal':[1,0],'candidates':['robot1']},
                    {'id':2,'goal':[-1,0],'candidates':['robot2']}]}
        a=self.planner.assign(s,m);batch=self.planner.next_batch(s,m,a,[])
        self.assertEqual(len(batch['actions']),1)
        self.assertIn(batch['audit']['strategy'],('relocate','short_replan'))
        self.assertGreaterEqual(len(batch['actions'][0]['points']),2)
        completed=[];intent=None
        for _ in range(12):
            batch=self.planner.next_batch(s,m,a,completed,intent)
            intent=batch.get('intent')
            if not batch['actions']:break
            for action in batch['actions']:
                s['robots'][action['robot']]['pose']=action['points'][-1]
                if action['task'] is not None:completed.append(action['task'])
        self.assertEqual(set(completed),{1,2})

    def test_worker_protocol_does_not_modify_marco(self):
        before=subprocess.check_output(['git','-C',str(ROOT/'MARCO'),'status','--porcelain'])
        directory=ROOT/'log/marco_test';directory.mkdir(parents=True,exist_ok=True)
        worker=Worker(ROOT/'MARCO',directory/'worker.log')
        try:
            result=worker.call({'command':'assign','snapshot':snapshot(),
                                'mission':{'tasks':[{'id':1,'goal':[1,0],'candidates':['robot1']}]}})
            self.assertEqual(result['assignment'],{'1':'robot1'})
        finally:worker.close()
        self.assertEqual(before,subprocess.check_output(['git','-C',str(ROOT/'MARCO'),'status','--porcelain']))

    def test_completion_racing_lease_renewal(self):
        action={'robot':'robot1','task':1,'points':[[0.,0.,0.],[.1,0.,0.]]}
        for terminal in ('succeeded','canceled'):
            status_calls=[]
            def rpc(value,port):
                command=value['command']
                if command=='snapshot':return {'map_revision':1,'robots':{'robot1':{'localized':True,'odom_fresh':True,'pose':[.1,0.,0.]}}}
                if command=='status':
                    status_calls.append(True)
                    return {'robots':{'robot1':{'busy':len(status_calls)==1,'last_result':terminal}}}
                if command=='lease':raise RuntimeError('Ownership already cleared')
                return {'ok':True}
            with patch('tb4_marco_bridge.cli.request',side_effect=rpc),patch('tb4_marco_bridge.cli.time.sleep'):
                if terminal=='succeeded':self.assertEqual(execute_batch([action],{},1,1,[]),[1])
                else:
                    with self.assertRaises(RuntimeError):execute_batch([action],{},1,1,[])

if __name__=='__main__':unittest.main()
