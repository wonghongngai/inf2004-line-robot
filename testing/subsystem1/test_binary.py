"""Independent wire fixtures and binary CLI tests (no hardware/broker)."""
import importlib.util
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
TOOL = ROOT / 'tools/mqtt_binary.py'
spec = importlib.util.spec_from_file_location('mqtt_binary', TOOL)
codec = importlib.util.module_from_spec(spec)
spec.loader.exec_module(codec)


class ProtocolTests(unittest.TestCase):
    def test_boundary_fixture(self):
        values = dict(zip((f[0] for f in codec.FIELDS),
                          [7, 31, 100, -123, 65535, 7, 4, 255, 400, 180, 400]))
        wire = bytes.fromhex('ffe426ffffe73f645ac8')
        self.assertEqual(codec.encode_telemetry(values), wire)
        self.assertEqual(codec.decode_telemetry(wire), values)

    def test_invalid_lengths_and_fields(self):
        for length in (0, 1, 9, 11):
            with self.assertRaises(ValueError):
                codec.decode_telemetry(bytes(length))
        with self.assertRaises(ValueError):
            codec.encode_telemetry({'speed_cms': 201})
        with self.assertRaises(ValueError):
            codec.encode_telemetry({'speed_cms': -201})
        with self.assertRaises(ValueError):
            codec.encode_telemetry({'unknown': 1})
        with self.assertRaises(ValueError):
            codec.decode_telemetry(bytes.fromhex('007f0000000000000000'))

    def test_raw_zero_command(self):
        result = subprocess.run([sys.executable, str(TOOL), 'command', 'status'],
                                check=True, capture_output=True)
        self.assertEqual(result.stdout, b'\x00')

    def test_display_stream(self):
        result = subprocess.run([sys.executable, str(TOOL), 'stream'],
                                input=b'ffe426ffffe73f645ac8\ninvalid\n',
                                check=True, capture_output=True)
        self.assertIn(b'"speed_cms": -123', result.stdout)
        self.assertIn(b'Rejected packet', result.stderr)


if __name__ == '__main__':
    unittest.main()
