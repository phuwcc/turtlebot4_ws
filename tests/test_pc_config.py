#!/usr/bin/env python3
"""No network or ROS needed: laptop input validation and atomic config updates."""
import contextlib
import importlib.util
import io
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('pc_config', Path(__file__).resolve().parents[1] / 'scripts/pc_config.py')
pc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pc)


class LaptopConfigTest(unittest.TestCase):
    def test_count_and_roundtrip(self):
        for count in (1, 2, 3):
            with self.subTest(count=count), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'config/pc_env.sh'
                addresses = [f'10.11.63.{119 + i}' for i in range(count)]
                values = pc.save_environment(path, addresses)
                self.assertEqual(pc.load_environment(path), values)
                self.assertEqual(values['TB4_ROBOT_COUNT'], str(count))
                self.assertEqual(values['ROS_DISCOVERY_SERVER'].count(':11811'), count)
                self.assertNotIn(';:', values['ROS_DISCOVERY_SERVER'])

    def test_invalid_addresses_do_not_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pc_env.sh'
            pc.save_environment(path, ['10.11.63.119'])
            original = path.read_text()
            for addresses in ([], [''], ['999.1.2.3'], ['10.11.63.119', ''],
                              ['127.0.0.1'], ['0.0.0.0'], ['224.1.2.3'], ['255.255.255.255'],
                              ['10.11.63.119'] * 2, ['10.0.0.1'] * 4,
                              ["10.1.2.3'; touch /tmp/never-execute"]):
                with self.subTest(addresses=addresses), self.assertRaises(ValueError):
                    pc.save_environment(path, addresses)
                self.assertEqual(path.read_text(), original)

    def test_prompt_retries_and_only_asks_two_robots(self):
        answers = iter(['', '4', '2', '', '999.0.0.1', '10.11.63.119',
                        '10.11.63.119', '10.11.52.137'])
        prompts = []
        def answer(prompt):
            prompts.append(prompt)
            return next(answers)
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            path = Path(directory) / 'pc_env.sh'
            pc.configure(path, input_fn=answer)
            self.assertEqual(pc.load_environment(path)['ROS_DISCOVERY_SERVER'],
                             '10.11.63.119:11811;10.11.52.137:11811;')
        self.assertFalse(any('robot3' in prompt for prompt in prompts))

    def test_interrupted_setup_preserves_previous_config(self):
        with tempfile.TemporaryDirectory() as directory, contextlib.redirect_stdout(io.StringIO()):
            path = Path(directory) / 'pc_env.sh'
            pc.save_environment(path, ['10.11.63.119'])
            original = path.read_text()
            answers = iter(['2', '10.11.63.119'])
            def answer(prompt):
                try:
                    return next(answers)
                except StopIteration:
                    raise EOFError()
            with self.assertRaises(EOFError):
                pc.configure(path, input_fn=answer)
            self.assertEqual(path.read_text(), original)

    def test_previous_broken_read_config_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pc_env.sh'
            path.write_text("export ROS_DISCOVERY_SERVER=':11811;:11811;:11811;'\n")
            with self.assertRaises(ValueError):
                pc.load_environment(path)


if __name__ == '__main__':
    unittest.main()
