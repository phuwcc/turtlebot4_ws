#!/usr/bin/env python3
"""Shutdown regression checks; never discover or signal real robot processes."""
import importlib.util
from pathlib import Path
import signal
import subprocess
import sys
import unittest

spec = importlib.util.spec_from_file_location('stop_hardware', Path(__file__).resolve().parents[1] / 'scripts/stop_hardware.py')
stop = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stop)


class HardwareStopTest(unittest.TestCase):
    def test_exact_launch_identity(self):
        argv = ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch',
                'tb4_multi_bringup', 'hardware.launch.py', 'robot_id:=robot1']
        self.assertTrue(stop.matches_hardware(argv, 'robot1'))
        self.assertFalse(stop.matches_hardware(argv, 'robot2'))
        self.assertFalse(stop.matches_hardware(['bash', '-c', ' '.join(argv)], 'robot1'))
        self.assertFalse(stop.matches_hardware(['flock', '-n', '/tmp/lock'] + argv[1:], 'robot1'))

    def test_orphans_are_scoped_to_robot_and_driver(self):
        argv = ['/opt/ros/humble/lib/create3_republisher/create3_republisher',
                '--ros-args', '-r', '__ns:=/robot1']
        self.assertTrue(stop.matches_hardware(argv, 'robot1'))
        self.assertFalse(stop.matches_hardware(argv, 'robot2'))
        self.assertFalse(stop.matches_hardware(['/usr/bin/other'] + argv[1:], 'robot1'))
        guard = ['python3', '/tmp/install/lib/tb4_multi_bringup/remote_velocity',
                 '--ros-args', '-r', '__ns:=/robot1', '-r', '__node:=remote_velocity_guard']
        self.assertTrue(stop.matches_hardware(guard, 'robot1'))
        guard[-1] = '__node:=remote_velocity_stamp'
        self.assertFalse(stop.matches_hardware(guard, 'robot1'))

    def test_sigterm_stops_child_ignoring_sigint(self):
        child = subprocess.Popen([sys.executable, '-c',
            'import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); '
            'print("ready",flush=True); time.sleep(30)'], stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(child.stdout.readline().strip(), 'ready')
            child.send_signal(signal.SIGINT)
            self.assertIsNone(child.poll())
            started = stop.identity(child.pid)
            self.assertIsNotNone(started)
            self.assertEqual(stop.terminate({child.pid: started}, timeout=3.), [])
            self.assertEqual(child.wait(timeout=3), -signal.SIGTERM)
        finally:
            if child.poll() is None:
                child.kill()
                child.wait()
            child.stdout.close()

    def test_changed_pid_identity_is_not_signaled(self):
        self.assertEqual(stop.terminate({123456789: 'different-start-time'}, timeout=.1), [])


if __name__ == '__main__':
    unittest.main()
