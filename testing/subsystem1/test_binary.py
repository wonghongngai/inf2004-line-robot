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
                          [7, 31, 7, -123, 65535, 7, 4, 255, 400, 180, 400, 6, 1, 65535, 65535, 1,
                           0x1234, 0xFEDC]))
        # bits 8-10 = S5 faults (#46); byte 10 bit 6 = map (#38); bytes 11-14 = two
        # 16-bit counters; bytes 15-18 = left/right ticks (#41)
        wire = bytes.fromhex('ff8726ffffe73f645ac866ffffffff3412dcfe')
        self.assertEqual(codec.encode_telemetry(values), wire)
        self.assertEqual(codec.decode_telemetry(wire), values)
        del values["left_ticks"]
        del values["right_ticks"]
        wire = wire[:15]                                   # 15-byte packet before #41
        self.assertEqual(codec.decode_telemetry(wire), values)
        del values["barcode_black"]
        del values["barcode_white"]
        wire = wire[:11]
        self.assertEqual(codec.decode_telemetry(wire), values)
        self.assertEqual(codec.decode_telemetry(wire[:-1] + b"\x06")["speed_valid"], 0)
        del values["speed_valid"]
        del values["barcode_letter"]
        del values["map_received"]
        self.assertEqual(codec.decode_telemetry(wire[:-1]), values)
        with self.assertRaises(ValueError):
            codec.decode_telemetry(wire[:-1] + b"\x1f")  # letter 31 > 26

    def test_s5_faults_and_reserved_bits(self):
        # #46: bits 8-10 = CAL_FAIL, NO_PATH, LINE_LOST; bits 11-14 and 87 reserved
        for value in range(8):
            wire = codec.encode_telemetry({'s5_faults': value})
            self.assertEqual(int.from_bytes(wire, 'little') >> 8 & 7, value)
            self.assertEqual(codec.decode_telemetry(wire)['s5_faults'], value)
        with self.assertRaises(ValueError):
            codec.encode_telemetry({'s5_faults': 8})
        with self.assertRaises(ValueError):
            codec.encode_telemetry({'barcode_black': 65536})
        wire = codec.encode_telemetry({'barcode_black': 1, 'barcode_white': 2})
        self.assertEqual(int.from_bytes(wire, 'little') >> 88, 1 | 2 << 16)
        for length in (15, codec.TELEMETRY_BYTES):
            for bit in (11, 12, 13, 14, 87):
                raw = bytearray(length)
                raw[bit // 8] |= 1 << (bit % 8)
                with self.assertRaises(ValueError):
                    codec.decode_telemetry(bytes(raw))

    def test_invalid_lengths_and_fields(self):
        for length in (0, 1, 9, 12, 16, 18, 20):
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

    def test_ticks_wrap(self):
        # #41: the PC reads 16-bit counts as int16 and differences them mod 2^16.
        self.assertEqual(codec.ticks_signed(0xFEDC), -292)
        self.assertEqual(codec.ticks_signed(0x7FFF), 32767)
        self.assertEqual(codec.ticks_signed(0x8000), -32768)
        self.assertEqual(codec.ticks_delta(0xFFF0, 0x0010), 32)     # forward wrap
        self.assertEqual(codec.ticks_delta(0x0010, 0xFFF0), -32)    # reverse wrap
        self.assertEqual(codec.ticks_delta(100, 2400), 2300)
        for count in (-70000, -1, 0, 1, 40000, 2**31 - 1):
            wire = codec.encode_telemetry({"left_ticks": count & 0xFFFF})
            self.assertEqual(codec.decode_telemetry(wire)["left_ticks"], count & 0xFFFF)

    def test_raw_zero_command(self):
        result = subprocess.run([sys.executable, str(TOOL), 'command', 'status'],
                                check=True, capture_output=True)
        self.assertEqual(result.stdout, b'\x00')

    def test_raw_start_command(self):
        # START (#48) is code 5, the first formerly reserved value.
        result = subprocess.run([sys.executable, str(TOOL), 'command', 'start'],
                                check=True, capture_output=True)
        self.assertEqual(result.stdout, b'\x05')

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
                                input=b'ff8726ffffe73f645ac8\ninvalid\n',
                                check=True, capture_output=True)
        self.assertIn(b'"speed_cms": -123', result.stdout)
        self.assertIn(b'Rejected packet', result.stderr)


if __name__ == '__main__':
    unittest.main()
