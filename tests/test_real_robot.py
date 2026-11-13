#!/usr/bin/env python3
"""Entry-point failure/cleanup contracts. No SSH or physical commands."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import real_robot as real
from pc_config import make_environment


class RealRobotTest(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.values = make_environment(['10.11.52.230', '10.11.52.243'], ['robot1', 'robot2'])

    def test_mapping_only_starts_selected_robot_and_saves_before_shutdown(self):
        args = real.parser().parse_args(['mapping', '--no-teleop', '--save-as', 'maps/new room'])
        child = Mock();child.poll.return_value = None;child.wait.side_effect = KeyboardInterrupt
        events = []
        with patch.object(real, 'require_pc'), patch.object(real, 'ensure_no_session'), \
                patch.object(real, 'start_hardware') as hardware, patch.object(real, 'preflight') as preflight, \
                patch.object(real, 'log_directory', return_value=self.directory), \
                patch.object(real.subprocess, 'Popen', return_value=child), \
                patch.object(real, 'checked', side_effect=lambda cmd: events.append(('save', cmd))), \
                patch.object(real, 'stop_child', side_effect=lambda proc: events.append(('stop', proc))):
            real.mapping(args, self.values, ['robot1', 'robot2'])
        hardware.assert_called_once_with(['robot1'], self.values, False)
        self.assertEqual(preflight.call_args_list[0].args[1]['TB4_ROBOT_IDS'], 'robot1')
        self.assertEqual(preflight.call_args_list[0].args[1]['ROS_DISCOVERY_SERVER'], '10.11.52.230:11811;')
        self.assertTrue(preflight.call_args_list[1].kwargs['map_ready'])
        self.assertEqual([event[0] for event in events], ['save', 'stop'])
        self.assertIn('pc-save-map.sh', events[0][1][1])
        self.assertEqual(events[0][1][-1], str(real.ROOT/'maps/new room'))

    def test_mapping_startup_failure_never_saves_an_unready_map(self):
        args = real.parser().parse_args(['mapping', '--no-teleop'])
        child = Mock()
        with patch.object(real, 'require_pc'), patch.object(real, 'ensure_no_session'), \
                patch.object(real, 'start_hardware'), \
                patch.object(real, 'preflight', side_effect=[None, RuntimeError('map missing')]), \
                patch.object(real, 'log_directory', return_value=self.directory), \
                patch.object(real.subprocess, 'Popen', return_value=child), \
                patch.object(real, 'checked') as checked, patch.object(real, 'stop_child') as stop:
            with self.assertRaisesRegex(RuntimeError, 'map missing'):
                real.mapping(args, self.values, ['robot1', 'robot2'])
        checked.assert_not_called();stop.assert_called_once_with(child)

    def test_incomplete_poses_fail_before_ssh(self):
        poses = self.directory/'poses.yaml'
        poses.write_text('robot1: {x: 0, y: 0, yaw: 0}\n')
        args = real.parser().parse_args(['multi', '--poses', str(poses)])
        with patch.object(real, 'require_pc'), patch.object(real, 'start_hardware') as hardware:
            with self.assertRaisesRegex(ValueError, 'robot2 requires'):
                real.multi(args, self.values, ['robot1', 'robot2'])
        hardware.assert_not_called()

    def test_preview_failure_never_dispatches_mission(self):
        with patch.object(real, 'checked', side_effect=subprocess.CalledProcessError(1, ['preview'])) as checked:
            with self.assertRaises(subprocess.CalledProcessError):
                real.mission_run(self.directory/'mission.json', True, self.directory)
        self.assertEqual(checked.call_count, 1)
        self.assertIn('plan', checked.call_args.args[0])
        self.assertNotIn('--execute', checked.call_args.args[0])

    def test_navigation_requires_localization_and_path_server_for_exact_fleet(self):
        statuses = [
            {'robot1': {'localized': True, 'path_ready': True}, 'robot2': {'localized': False, 'path_ready': True}},
            {'robot1': {'localized': True, 'path_ready': True}, 'robot2': {'localized': True, 'path_ready': False}},
            {r: {'localized': True, 'path_ready': True} for r in ('robot1', 'robot2', 'robot3')},
        ]
        child = Mock();child.poll.return_value = None
        for status in statuses:
            with self.subTest(status=status), \
                    patch('socket_client.send', return_value={'ok': True, 'robots': status}), \
                    patch.object(real.time, 'monotonic', side_effect=[0., 0., 1.]), patch.object(real.time, 'sleep'):
                with self.assertRaisesRegex(RuntimeError, 'Chưa đạt định vị'):
                    real.wait_navigation(child, ['robot1', 'robot2'], .1)

    def test_execute_without_mission_is_rejected_before_any_robot_access(self):
        with patch.object(real, 'fleet') as fleet:
            with self.assertRaises(SystemExit) as context:
                real.main(['multi', '--execute'])
        self.assertEqual(context.exception.code, 2);fleet.assert_not_called()

    def test_ssh_honors_nonstandard_discovery_slots_and_quotes_remote_arguments(self):
        values = make_environment(['10.11.52.243'], ['robot2'], [0])
        with patch.dict(real.os.environ, {'TB4_REMOTE_REPO': '/home/ubuntu/my workspace'}, clear=True):
            argv = real.ssh_command('robot2', values, ['bash', 'scripts/check.sh', 'literal $(touch BAD);'])
        self.assertEqual(argv[-2], 'ubuntu@10.11.52.243')
        self.assertIn("cd '/home/ubuntu/my workspace'", argv[-1])
        self.assertTrue(argv[-1].endswith("'literal $(touch BAD);'"))


if __name__ == '__main__':
    unittest.main()
