"""Pure launch/configuration tests. Runtime C++ is tested in test_native_runtime.py."""
from pathlib import Path as FilePath
import tempfile
import unittest
import yaml
from tb4_multi_bringup.fleet_settings import ROBOTS, active_robots, rviz_for_count, startup_poses, validate_discovery


class FleetConfigurationTest(unittest.TestCase):
    def test_startup_poses(self):
        self.assertEqual(startup_poses('', 2), {})
        with tempfile.TemporaryDirectory() as directory:
            path = FilePath(directory) / 'poses.yaml'
            path.write_text('robot1: {x: 1, y: 2, yaw: 0.5}\n'
                            'robot2: {x: -3, y: 4, yaw: -1}\n')
            poses = startup_poses(str(path), 2)
            self.assertEqual(poses['robot1']['initial_x'], '1')
            self.assertEqual(poses['robot2']['initial_x'], '-3')
            selected = startup_poses(str(path), 1, 'robot2')
            self.assertEqual(set(selected), {'robot2'})
            self.assertEqual(selected['robot2']['initial_x'], '-3')
            for text in ('[]', 'robot1: {x: 1, y: 2}',
                         'robot1: {x: .nan, y: 2, yaw: 0}',
                         'robot1: {x: true, y: 2, yaw: 0}',
                         'robot1: {x: 1, y: 2, yaw: 0}',
                         'robot4: {x: 1, y: 2, yaw: 0}'):
                path.write_text(text)
                with self.subTest(text=text), self.assertRaises(ValueError):
                    startup_poses(str(path), 2)

    def test_configuration_filters_and_rejects_broken_discovery(self):
        template = yaml.safe_load((FilePath(__file__).resolve().parents[1] /
                                   'config/fleet.rviz').read_text())
        for count in (1, 2, 3):
            view = rviz_for_count(template, count)
            groups = [d['Name'] for d in view['Visualization Manager']['Displays'] if d['Name'] in ROBOTS]
            self.assertEqual(groups, list(ROBOTS[:count]))
            valid = ';'.join(f'10.11.63.{119 + index}:11811' for index in range(count)) + ';'
            validate_discovery(valid, count)
        self.assertEqual(len([d for d in template['Visualization Manager']['Displays']
                              if d['Name'] in ROBOTS]), 3)
        for value, count in ((':11811;:11811;:11811;', 3), ('', 1),
                             ('10.11.63.119:11811;:11811;', 2),
                             ('999.0.0.1:11811;', 1), ('10.11.63.119:11811;', 2)):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_discovery(value, count)
        for count in (0, 4, True):
            with self.assertRaises(ValueError):
                active_robots(count)

    def test_light_rviz_defaults_preserve_robot_models(self):
        view = yaml.safe_load((FilePath(__file__).resolve().parents[1] / 'config/fleet.rviz').read_text())
        manager = view['Visualization Manager']
        self.assertLessEqual(manager['Global Options']['Frame Rate'], 10)
        self.assertFalse(next(d for d in manager['Displays'] if d['Name'] == 'Robot TF trees')['Enabled'])
        self.assertTrue(next(d for d in manager['Displays'] if d['Name'].startswith('Shared map'))['Enabled'])
        for group in manager['Displays']:
            if group.get('Name') in ROBOTS:
                model = next(d for d in group['Displays'] if d['Class'].endswith('/RobotModel'))
                self.assertTrue(model['Enabled'])
                self.assertTrue(model['Visual Enabled'])

    def test_robot2_only_keeps_namespace_map_and_discovery_slot(self):
        self.assertEqual(active_robots(1, 'robot2'), ('robot2',))
        validate_discovery(';10.11.52.137:11811;', 1, 'robot2')
        validate_discovery('10.11.52.137:11811;', 1, 'robot2', '0')
        with self.assertRaises(ValueError):
            validate_discovery('10.11.52.137:11811;', 1, 'robot2')
        template = yaml.safe_load((FilePath(__file__).resolve().parents[1] / 'config/fleet.rviz').read_text())
        view = rviz_for_count(template, 1, 'robot2')
        displays = view['Visualization Manager']['Displays']
        self.assertEqual([d['Name'] for d in displays if d['Name'] in ROBOTS], ['robot2'])
        self.assertEqual(next(d for d in displays if d['Name'].startswith('Shared map'))['Topic']['Value'],
                         '/tb4_view/robot2/map')


if __name__ == '__main__':
    unittest.main()
