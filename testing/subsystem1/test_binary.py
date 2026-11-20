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
                          [7, 31, 100, -123, 65535, 7, 4, 255, 400, 180, 400, 6, 1, 15, 14]))
        wire = bytes.fromhex('ffe426ffffe73f645ac8260f000e00')
        self.assertEqual(codec.encode_telemetry(values), wire)
        self.assertEqual(codec.decode_telemetry(wire), values)
        del values["barcode_black"]
        del values["barcode_white"]
        wire = wire[:11]
        self.assertEqual(codec.decode_telemetry(wire), values)
        self.assertEqual(codec.decode_telemetry(wire[:-1] + b"\x06")["speed_valid"], 0)
        del values["speed_valid"]
        del values["barcode_letter"]
        self.assertEqual(codec.decode_telemetry(wire[:-1]), values)
        with self.assertRaises(ValueError):
            codec.decode_telemetry(wire[:-1] + b"\xe6")

    def test_invalid_lengths_and_fields(self):
        for length in (0, 1, 9, 12):
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

    def test_raw_barcode_mapping(self):
        result = subprocess.run(
            [sys.executable, str(TOOL), 'mapping', '--left', 'A', '--right', 'B',
             '--straight', 'C', '--uturn', 'Z'],
            check=True, capture_output=True,
        )
        self.assertEqual(result.stdout, bytes.fromhex('20 88 0c'))
        for letters in (('A', 'A', 'C', 'D'), ('A', 'B', 'C', 'ZZ')):
            with self.assertRaises(ValueError):
                codec.encode_mapping(letters)

    def test_barcode_result_fixture(self):
        # Same fixtures as protocol_test.c: the C and Python codecs must agree.
        self.assertEqual(codec.encode_barcode(1, "Z", 29), bytes.fromhex("0100ba03"))
        self.assertEqual(codec.decode_barcode(bytes.fromhex("0100ba03")),
                         {"frame": 1, "status": "valid", "letter": "Z", "runs": 29})
        self.assertEqual(codec.encode_barcode(0xFFFF, None, 23), bytes.fromhex("ffffe002"))
        self.assertEqual(codec.decode_barcode(bytes.fromhex("ffffe002")),
                         {"frame": 65535, "status": "invalid", "letter": None, "runs": 23})
        self.assertEqual(codec.decode_barcode(codec.encode_barcode(0x10001, "A", 0))["frame"], 1)
        for letter in [None] + [chr(c) for c in range(ord("A"), ord("Z") + 1)]:
            for runs in range(30):
                wire = codec.encode_barcode(7, letter, runs)
                self.assertEqual(codec.decode_barcode(wire)["letter"], letter)
                self.assertEqual(codec.decode_barcode(wire)["runs"], runs)

    def test_barcode_result_rejects(self):
        for payload in (b"", bytes(3), bytes(5)):
            with self.assertRaises(ValueError):
                codec.decode_barcode(payload)
        for bit in range(26, 32):
            with self.assertRaises(ValueError):
                codec.decode_barcode((0x03BA0001 | 1 << bit).to_bytes(4, "little"))
        with self.assertRaises(ValueError):
            codec.decode_barcode((27 << 16).to_bytes(4, "little"))   # letter 27
        with self.assertRaises(ValueError):
            codec.decode_barcode((31 << 21).to_bytes(4, "little"))   # runs 31
        for args in ((1, "?", 29), (1, "Z", 30), (-1, "Z", 1), (1, "ZZ", 1)):
            with self.assertRaises(ValueError):
                codec.encode_barcode(*args)

    def test_barcode_stream(self):
        result = subprocess.run([sys.executable, str(TOOL), 'barcode-stream'],
                                input=b'0100ba03\nffffffff\n',
                                check=True, capture_output=True)
        self.assertIn(b'"letter": "Z"', result.stdout)
        self.assertIn(b'Rejected packet', result.stderr)

    def test_display_stream(self):
        result = subprocess.run([sys.executable, str(TOOL), 'stream'],
                                input=b'ffe426ffffe73f645ac8\ninvalid\n',
                                check=True, capture_output=True)
        self.assertIn(b'"speed_cms": -123', result.stdout)
        self.assertIn(b'Rejected packet', result.stderr)


if __name__ == '__main__':
    unittest.main()
