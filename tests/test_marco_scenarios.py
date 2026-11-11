"""Exercise documented conflict missions with MARCO on synthetic maps, no ROS."""
import copy
import json
import math
from pathlib import Path
import unittest

from tb4_marco_bridge.planning import Planner


ROOT = Path(__file__).resolve().parents[1]
CASES = ROOT / 'config/marco/testcases'


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
    def test_conflict_cases_complete_in_bounded_batches(self):
        for name in ('04_crossing', '05_head_on', '06_idle_blocker', '07_conflict_dag', '09_head_on_1m'):
            with self.subTest(name=name):
                planner = Planner(ROOT / 'MARCO')
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
        planner = Planner(ROOT / 'MARCO')
        mission = json.loads((CASES / '08_no_escape.json').read_text())
        state = scenario('08_no_escape')
        before = copy.deepcopy(state)
        assignment = planner.assign(state, mission)
        with self.assertRaisesRegex(RuntimeError, 'No feasible guarded MARCO intervention|no complete conflict-free relocation plan'):
            planner.next_batch(state, mission, assignment, [])
        self.assertEqual(state, before)


if __name__ == '__main__':
    unittest.main()
