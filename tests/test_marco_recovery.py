import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from tb4_marco_bridge.cli import CorridorViolation, LocalizationUnavailable, execute_batch, main, wait_recovery

class RecoveryTest(unittest.TestCase):
    def test_recovery_requires_stopped_ready_fleet(self):
        phases=[(False,[0.,0.],False),(True,[.02,0.],False),(True,[0.,0.],True),(True,[0.,0.],False)]
        calls=[]
        def rpc(q,port):
            phase=phases[min(len(calls),3)]
            if q['command']=='snapshot':
                calls.append(q);phase=phases[min(len(calls)-1,3)]
                return {'map_revision':1,'robots':{'robot1':{'localized':phase[0],'odom_fresh':True,'pose':[0.,0.,0.],'velocity':phase[1]}}}
            return {'robots':{'robot1':{'busy':phases[min(len(calls)-1,3)][2],'path_ready':True}}}
        with patch('tb4_marco_bridge.cli.request',side_effect=rpc),patch('tb4_marco_bridge.cli.time.sleep'):
            result=wait_recovery(1,1,['robot1'],stable_seconds=0)
        self.assertEqual(len(calls),4);self.assertTrue(result['robots']['robot1']['localized'])

    def test_map_change_never_resumes(self):
        with patch('tb4_marco_bridge.cli.request',return_value={'map_revision':2}),self.assertRaisesRegex(RuntimeError,'Map changed'):
            wait_recovery(1,1,['robot1'])

    def test_missing_velocity_never_resumes(self):
        def rpc(q,port):
            if q['command']=='snapshot':return {'map_revision':1,'robots':{'robot1':{'localized':True,'odom_fresh':True,'pose':[0.,0.,0.]}}}
            return {'robots':{'robot1':{'busy':False,'path_ready':True}}}
        with patch('tb4_marco_bridge.cli.request',side_effect=rpc),patch('tb4_marco_bridge.cli.time.sleep'),self.assertRaises(TimeoutError):
            wait_recovery(1,1,['robot1'],timeout=.01,stable_seconds=0)

    def test_corridor_stop_cancels_and_logs_trip_pose(self):
        action = {'robot': 'robot2', 'task': None, 'points': [[0, .02, 0], [1, .02, 0]],
                  'reserved': [[0, 0, 0], [1, 0, 0]]}
        snapshot = {'map_revision': 1, 'robots': {'robot2': {
            'localized': True, 'odom_fresh': True, 'pose': [.2, .065, 0], 'velocity': [.06, 0]}}}
        events = []
        with patch('tb4_marco_bridge.cli.request',
                   side_effect=[{'ok': True}, snapshot, {'robots': {'robot2': {'busy': True}}}, {'ok': True}]) as rpc:
            with self.assertRaisesRegex(CorridorViolation, '0.065 m > 0.060 m'):
                execute_batch([action], {'tracking_tolerance': .06}, 1, 1, events)
        self.assertEqual(rpc.call_args.args[0], {'command': 'cancel', 'robot': 'robot2'})
        self.assertEqual(events[-1]['event'], 'corridor_violation')
        self.assertEqual(events[-1]['pose'], [.2, .065, 0])

    def test_stale_odom_cancels_and_logs_during_relocation(self):
        action = {'robot': 'robot1', 'task': None, 'points': [[0, 0, 0], [1, 0, 0]]}
        state = {'localized': False, 'odom_fresh': False, 'pose': None,
                 'localization': {'reason': 'stale_sensors', 'odom_receipt_age_s': .6}}
        snapshot = {'map_revision': 1, 'robots': {'robot1': state}}
        events = []
        with patch('tb4_marco_bridge.cli.request',
                   side_effect=[{'ok': True}, snapshot, {'ok': True}]) as rpc:
            with self.assertRaises(LocalizationUnavailable) as failure:
                execute_batch([action], {}, 1, 1, events)
        self.assertTrue(failure.exception.is_stale)
        self.assertEqual(rpc.call_args.args[0], {'command': 'cancel', 'robot': 'robot1'})
        self.assertEqual(events[-1], {'event': 'localization_unavailable', 'robot': 'robot1', 'state': state})

    def test_file_mission_replans_once_after_relocation_stops(self):
        start = {'map_revision': 1, 'map': {}, 'robots': {'robot2': {
            'localized': True, 'odom_fresh': True, 'pose': [0, 0, 0]}}}
        stopped = {**start, 'robots': {'robot2': {**start['robots']['robot2'], 'pose': [.2, .065, 0]}}}
        mission = {'tasks': [{'id': 2, 'goal': [1, 0, 0], 'candidates': ['robot2']}]}
        batch = {'actions': [{'robot': 'robot2', 'task': None, 'points': [[0, 0, 0], [1, 0, 0]]}],
                 'intent': {'relocations': []}}
        cases = [
            (CorridorViolation('corridor'), 'corridor_replan'),
            (LocalizationUnavailable('robot2', {'localization': {'reason': 'stale_sensors'}}),
             'localization_replan'),
        ]
        for error, event in cases:
            with self.subTest(event=event), TemporaryDirectory() as directory:
                path, out = Path(directory) / 'mission.json', Path(directory) / 'execution.json'
                path.write_text(json.dumps(mission))
                with patch('sys.argv', ['marco', 'run', str(path), '--execute', '--out', str(out)]), \
                     patch('tb4_marco_bridge.cli.request', return_value=start), \
                     patch('tb4_marco_bridge.cli.wait_for_fleet', return_value=start), \
                     patch('tb4_marco_bridge.cli.wait_recovery', return_value=stopped) as recover, \
                     patch('tb4_marco_bridge.cli.execute_batch', side_effect=error), \
                     patch('tb4_marco_bridge.cli.Worker') as worker:
                    worker.return_value.call.side_effect = [{}, batch, batch]
                    with self.assertRaises(type(error)):
                        main()
                recover.assert_called_once()
                replan = worker.return_value.call.call_args.args[0]
                self.assertIsNone(replan['intent'])
                self.assertEqual(replan['snapshot']['robots']['robot2']['pose'], [.2, .065, 0])
                self.assertEqual(replan['mission'], mission)
                report = json.loads(out.read_text())
                self.assertEqual(report['completed_tasks'], [])
                self.assertEqual([e['event'] for e in report['events']], ['plan', event, 'plan'])


if __name__=='__main__':unittest.main()
