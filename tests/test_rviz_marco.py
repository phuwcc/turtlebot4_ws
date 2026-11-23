#!/usr/bin/env python3
"""Staged RViz goals must never dispatch until explicit successful preview."""
import os
os.environ.update(ROS_DOMAIN_ID='183',ROS_LOCALHOST_ONLY='1',ROS_SUPER_CLIENT='False')
os.environ.pop('ROS_DISCOVERY_SERVER',None)
import tempfile
import unittest
from unittest.mock import patch,Mock
import rclpy
from geometry_msgs.msg import PoseStamped
from tb4_marco_bridge.rviz_goals import RvizGoals

class RvizMarcoTest(unittest.TestCase):
    def test_staging_preview_execute_and_edit_invalidation(self):
        rclpy.init()
        with tempfile.TemporaryDirectory() as root:
            app=RvizGoals(root)
            try:
                p=PoseStamped();p.header.frame_id='map';p.pose.orientation.w=1.
                with patch('tb4_marco_bridge.rviz_goals.subprocess.Popen') as spawn:
                    app.goal('robot1',p);app.command('execute');spawn.assert_not_called()
                    p.pose.position.x=1.;app.goal('robot2',p)
                    app.command('execute');spawn.assert_not_called()
                    child=Mock();child.poll.return_value=0;child.returncode=0;spawn.return_value=child
                    app.command('preview');self.assertEqual(spawn.call_count,1)
                    self.assertNotIn('--execute',spawn.call_args.args[0])
                    app.poll();self.assertTrue(app.preview_ok)
                    app.goal('robot2',p);self.assertFalse(app.preview_ok)
                    app.command('execute');self.assertEqual(spawn.call_count,1)
                    app.command('preview');app.poll();app.command('execute')
                    self.assertIn('--execute',spawn.call_args.args[0]);app.poll()
                    child.returncode=1
                    app.command('preview')
                    app.stream.write('RuntimeError: Task 1 goal is not footprint-safe\n')
                    app.poll()
                    self.assertIn('Task 1 goal is not footprint-safe',app.text)
                    self.assertFalse(app.preview_ok)
                    count=spawn.call_count;app.command('execute')
                    self.assertEqual(spawn.call_count,count)
            finally:app.close();rclpy.shutdown()

if __name__=='__main__':unittest.main()
