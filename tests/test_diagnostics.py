#!/usr/bin/env python3
"""Map startup validation without connecting to ROS or a robot."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('pc_diagnose', Path(__file__).resolve().parents[1] / 'scripts/pc_diagnose.py')
diag = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diag)


class MapValidationTest(unittest.TestCase):
    def test_relative_and_absolute_images(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            image = root / 'room.pgm'
            image.write_bytes(b'P5\n1 1\n255\n\xff')
            config = root / 'room.yaml'
            for value in ('room.pgm', str(image)):
                config.write_text('image: ' + value + '\n')
                self.assertEqual(diag.validate_map(config)['image'], str(image))

    def test_missing_map_and_invalid_image(self):
        with tempfile.TemporaryDirectory() as directory:
            config = Path(directory) / 'room.yaml'
            with self.assertRaises(ValueError):
                diag.validate_map(config)
            for text in ('[]', '{}', 'image: null', 'image: 7', 'image: missing.pgm'):
                config.write_text(text)
                with self.subTest(text=text), self.assertRaises(ValueError):
                    diag.validate_map(config)


if __name__ == '__main__':
    unittest.main()
