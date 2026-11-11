"""Small startup-menu tests. No ROS, SSH, sudo or motor commands are executed."""
import contextlib
import io
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import run as menu


class RunMenuTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.repo = Path(self.temp.name)
        (self.repo/'maps').mkdir()
        (self.repo/'maps/my_room.yaml').write_text('image: my_room.pgm\n')
        (self.repo/'poses.yaml').write_text('robot1: {x: 1, y: 2, yaw: 0}\n')
        (self.repo/'mission.json').write_text('{"tasks":[{"id":1,"goal":[0,0]}]}')
        self.quiet = contextlib.redirect_stdout(io.StringIO())
        self.quiet.__enter__()
        self.addCleanup(self.quiet.__exit__, None, None, None)

    def answers(self, *values):
        answers = iter(values)
        return lambda prompt: next(answers)

    def test_robot_selection_and_hardware_route(self):
        with patch.dict(menu.os.environ, {'ROBOT_NAMESPACE':'/robot2'}):
            robot = menu.robot_choice(['robot1','robot2'], self.answers('9',''))
        self.assertEqual(robot, 'robot2')
        self.assertEqual(menu.robot_command(self.repo,'1',robot)[-2:], [str(self.repo/'scripts/robot-start.sh'),'robot2'])
        self.assertEqual(menu.robot_command(self.repo,'2',robot)[-2:], [str(self.repo/'scripts/robot-teleop.sh'),'robot2'])
        with self.assertRaises(ValueError): menu.robot_command(self.repo,'1','robot9')
        with self.assertRaises(ValueError): menu.robot_command(self.repo,'8','robot1')

    def test_map_retry_and_explicit_current_pose(self):
        command = menu.pc_command(self.repo,'1',['robot1'],self.answers('missing.yaml','','','2','poses.yaml'))
        self.assertEqual(command[1],str(self.repo/'scripts/pc-marco.sh'))
        self.assertIn('initial_poses:='+str(self.repo/'poses.yaml'),command)
        self.assertNotIn('localization_only:=true',command)

    def test_localization_has_no_mission_execution(self):
        command = menu.pc_command(self.repo,'2',['robot1','robot2'],self.answers('','1'))
        self.assertIn('localization_only:=true',command)
        self.assertNotIn('--execute',command)

    def test_mapping_uses_selected_robot_only(self):
        command = menu.pc_command(self.repo,'3',['robot2'],self.answers(''))
        self.assertEqual(command[-1],'robot2')
        self.assertEqual(command[1],str(self.repo/'scripts/pc-mapping.sh'))

    def test_mapping_defaults_to_robot1_even_with_robot2_namespace(self):
        with patch.dict(menu.os.environ, {'ROBOT_NAMESPACE':'/robot2'}):
            command = menu.pc_command(self.repo,'3',['robot1','robot2'],self.answers(''))
        self.assertEqual(command[-1], 'robot1')
        command = menu.pc_command(self.repo,'3',['robot1','robot2'],self.answers('2'))
        self.assertEqual(command[-1], 'robot2')

    def test_preview_failure_never_returns_execute_command(self):
        with patch.object(menu.subprocess,'run',side_effect=subprocess.CalledProcessError(1,['preview'])) as call:
            with self.assertRaises(subprocess.CalledProcessError):
                menu.pc_command(self.repo,'6',['robot1'],self.answers('mission.json'))
        self.assertEqual(call.call_args.args[0][-2:],['plan',str(self.repo/'mission.json')])

    def test_execution_requires_preview_and_explicit_answer(self):
        with patch.object(menu.subprocess,'run') as call:
            with self.assertRaises(ValueError):
                menu.pc_command(self.repo,'6',['robot1'],self.answers('mission.json',''))
            command = menu.pc_command(self.repo,'6',['robot1'],self.answers('mission.json','CHAY'))
        self.assertEqual(call.call_count,2)
        self.assertEqual(command[-3:],['run',str(self.repo/'mission.json'),'--execute'])

    def test_bad_json_and_declined_build_do_not_start_processes(self):
        (self.repo/'bad.json').write_text('{"tasks":[]}')
        with patch.object(menu.subprocess,'run') as call:
            with self.assertRaises(ValueError):menu.pc_command(self.repo,'6',['robot1'],self.answers('bad.json'))
            with self.assertRaises(ValueError):menu.ensure_built(self.repo,'pc',self.answers(''))
        call.assert_not_called()

    def test_eof_does_not_select_default(self):
        def interrupted(prompt): raise EOFError()
        with self.assertRaises(EOFError):menu.choose('Mode',{'1':'hardware'},interrupted,default='1')

    def test_paths_are_literal_arguments(self):
        name='mission $(touch SHOULD_NOT_EXIST).json'
        (self.repo/name).write_text('{"tasks":[{"id":1}]}')
        command=menu.pc_command(self.repo,'5',['robot1'],self.answers(name))
        self.assertEqual(command[-1],str(self.repo/name))
        self.assertEqual(command[0],'bash')
        self.assertFalse((self.repo/'SHOULD_NOT_EXIST').exists())

    def test_missing_submodule_is_optional_and_checked(self):
        with patch.dict(menu.os.environ, {'MARCO_ROOT':str(self.repo/'MARCO')}),patch.object(menu.subprocess,'run') as call:
            with self.assertRaises(ValueError):menu.ensure_marco(self.repo,self.answers(''))
            call.assert_not_called()
            with self.assertRaises(ValueError):menu.ensure_marco(self.repo,self.answers('y'))
            self.assertEqual(call.call_count,1)

    def test_existing_submodule_requires_no_network(self):
        path=self.repo/'MARCO/src/core/assignment.py';path.parent.mkdir(parents=True);path.write_text('')
        with patch.dict(menu.os.environ, {'MARCO_ROOT':str(self.repo/'MARCO')}),patch.object(menu.subprocess,'run') as call:
            menu.ensure_marco(self.repo,self.answers())
        call.assert_not_called()

    def test_entry_points_cancel_cleanly_without_input_from_other_directory(self):
        root=Path(__file__).resolve().parents[1]
        for entry in ('run_robot.sh','run_pc.sh'):
            result=subprocess.run(['bash',str(root/entry)],cwd=self.repo,input='',
                                  capture_output=True,text=True,timeout=5)
            self.assertEqual(result.returncode,1,result.stderr)
            self.assertIn('Chưa khởi động tác vụ mới',result.stdout)
            self.assertNotIn('Traceback',result.stderr)


if __name__=='__main__':unittest.main()
