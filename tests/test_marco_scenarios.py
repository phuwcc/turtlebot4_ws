"""Exercise documented conflict missions with MARCO on synthetic maps, no ROS."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest

from tb4_marco_bridge.planning import Planner


ROOT = Path(__file__).resolve().parents[1]
CASES = ROOT / 'config/marco/testcases'
sys.path.insert(0, str(ROOT / 'scripts'))
from capture_marco_swap import swap_mission


def scenario(name):
    poses = {'robot1': [-1., 0., 0.], 'robot2': [1., 0., math.pi]}
    if name == '04_crossing':
        poses['robot2'] = [0., -1., math.pi / 2]
    elif name == '06_idle_blocker':
        poses['robot2'] = [0., 0., 0.]
    elif name == '09_head_on_1m':
        poses = {'robot1': [0., 0., math.pi], 'robot2': [-1., 0., 0.]}
    cells = []
    for y in range(41):
        for x in range(41):
            blocked = x in (0, 40) or y in (0, 40)
            if name == '08_no_escape':
                blocked = blocked or abs(y - 20) > 4 or abs(x - 20) > 13
            cells.append(100 if blocked else 0)
    return {
        'map': {'width': 41, 'height': 41, 'resolution': .1,
                'origin': [-2.05, -2.05, 0.], 'data': cells},
        'robots': {robot: {'pose': pose} for robot, pose in poses.items()},
    }


class DocumentedScenarioTest(unittest.TestCase):
    def test_live_pose_swap_yields_to_robot1_and_keeps_captured_goals(self):
        state = scenario('09_head_on_1m')
        state['ok'] = True
        for robot in state['robots'].values():
            robot.update(localized=True, odom_fresh=True, busy=False)
        initial = copy.deepcopy(state['robots'])
        mission = swap_mission(state)
        planner = Planner(ROOT / 'MACRO')
        assignment = planner.assign(state, mission)
        completed, intent = [], None
        yielded = False
        for _ in range(32):
            batch = planner.next_batch(state, mission, assignment, completed, intent)
            intent = batch.get('intent')
            if not batch['actions']:
                self.assertEqual(batch['reason'], 'completed')
                break
            for action in batch['actions']:
                if action['robot'] == 'robot2' and action['task'] is None and not completed:
                    yielded = True
                state['robots'][action['robot']]['pose'] = action['points'][-1]
                if action['task'] is not None:
                    completed.append(action['task'])
        else:
            self.fail('Swap did not finish within the batch limit')
        self.assertTrue(yielded)
        self.assertEqual(completed, [1, 2])
        self.assertEqual(state['robots']['robot1']['pose'], initial['robot2']['pose'])
        self.assertEqual(state['robots']['robot2']['pose'], initial['robot1']['pose'])

    def test_fixed_robot1_priority_and_idle_robot_protection(self):
        for name in ('04_crossing','05_head_on','06_idle_blocker','09_head_on_1m'):
            with self.subTest(name=name):
                state=scenario(name);mission=json.loads((CASES/(name+'.json')).read_text())
                mission.update(priority_order=['robot1','robot2'],speed_mps=.12,tracking_tolerance=.08)
                planner=Planner(ROOT/'MACRO');assignment=planner.assign(state,mission)
                completed=[];intent=None
                for _ in range(32):
                    batch=planner.next_batch(state,mission,assignment,completed,intent);intent=batch.get('intent')
                    if not batch['actions']:break
                    if batch['audit'].get('priority_robot') is not None:
                        expected='R1' if 1 not in completed else 'R2'
                        self.assertEqual(batch['audit']['priority_robot'],expected)
                    for action in batch['actions']:
                        if 1 not in completed and action['robot']=='robot2':self.assertIsNone(action['task'])
                        state['robots'][action['robot']]['pose']=action['points'][-1]
                        if action['task'] is not None:completed.append(action['task'])
                self.assertEqual(set(completed),{t['id'] for t in mission['tasks']})
        # A lower-priority goal cannot evict robot1 from its occupied position.
        state=scenario('05_head_on')
        mission={'priority_order':['robot1','robot2'],'tasks':[{'id':2,'goal':[-1,0,0],'candidates':['robot2']}]}
        planner=Planner(ROOT/'MACRO');assignment=planner.assign(state,mission)
        with self.assertRaises(RuntimeError):planner.next_batch(state,mission,assignment,[])

    def test_conflict_cases_complete_in_bounded_batches(self):
        for name in ('04_crossing', '05_head_on', '06_idle_blocker', '07_conflict_dag', '09_head_on_1m'):
            with self.subTest(name=name):
                planner = Planner(ROOT / 'MACRO')
                mission = json.loads((CASES / (name + '.json')).read_text())
                state = scenario(name)
                assignment = planner.assign(state, mission)
                completed, intent, strategies = [], None, []
                for _ in range(32):
                    batch = planner.next_batch(state, mission, assignment, completed, intent)
                    intent = batch.get('intent')
                    if not batch['actions']:
                        self.assertEqual(batch['reason'], 'completed')
                        break
                    strategies.append(batch['audit']['strategy'])
                    before = set(completed)
                    for action in batch['actions']:
                        if action['task'] is not None:
                            task = next(t for t in mission['tasks'] if t['id'] == action['task'])
                            self.assertTrue(set(task['depends_on']) <= before)
                            completed.append(action['task'])
                        state['robots'][action['robot']]['pose'] = copy.deepcopy(action['points'][-1])
                else:
                    self.fail(name + ' exceeded the bounded batch limit')
                self.assertEqual(set(completed), {t['id'] for t in mission['tasks']})
                self.assertNotEqual(strategies[0], 'disjoint_routes')
                if name == '09_head_on_1m':
                    for task in mission['tasks']:
                        actual = state['robots'][task['candidates'][0]]['pose']
                        self.assertLess(math.dist(actual[:2], task['goal'][:2]), .1)

    def test_no_escape_rejects_before_any_action(self):
        planner = Planner(ROOT / 'MACRO')
        mission = json.loads((CASES / '08_no_escape.json').read_text())
        state = scenario('08_no_escape')
        before = copy.deepcopy(state)
        assignment = planner.assign(state, mission)
        with self.assertRaisesRegex(RuntimeError, 'No feasible guarded MARCO intervention|no complete conflict-free relocation plan'):
            planner.next_batch(state, mission, assignment, [])
        self.assertEqual(state, before)


if __name__ == '__main__':
    unittest.main()
