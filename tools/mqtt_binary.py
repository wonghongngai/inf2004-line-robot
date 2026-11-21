#!/usr/bin/env python3
"""Protocol v1 helper for binary telemetry, commands, barcode maps and results."""
import argparse
import json
import sys

# name, absolute bit offset, width, logical minimum, logical maximum
FIELDS = (
    ("state", 0, 3, 0, 7), ("faults", 3, 5, 0, 31),
    ("battery_percent", 8, 7, 0, 100), ("speed_cms", 15, 9, -200, 200),
    ("distance_cm", 24, 16, 0, 65535), ("line_sensors", 40, 3, 0, 7),
    ("barcode", 43, 3, 0, 4), ("highest_hump_mm", 46, 8, 0, 255),
    ("obstacle_cm", 54, 9, 0, 400), ("scan_deg", 63, 8, 0, 180),
    ("obstacle_width_cm", 71, 9, 0, 400),
    ("barcode_letter", 80, 5, 0, 26), ("speed_valid", 85, 1, 0, 1),
    ("barcode_black", 88, 16, 0, 65535), ("barcode_white", 104, 16, 0, 65535),
)
COMMANDS = {"status": 0, "pause": 1, "resume": 2, "reset-stats": 3, "reverse": 4, "start": 5}


def encode_mapping(letters):
    if len(letters) != 4 or any(len(letter) != 1 or not "A" <= letter <= "Z"
                                for letter in letters):
        raise ValueError("provide one uppercase letter A–Z for each action")
    if len(set(letters)) != 4:
        raise ValueError("barcode map letters must be distinct")
    packed = sum((ord(letter) - ord("A")) << (5 * slot)
                 for slot, letter in enumerate(letters))
    return packed.to_bytes(3, "little")


def decode_telemetry(payload):
    if len(payload) not in (10, 11, 15):
        raise ValueError("telemetry must be 10, 11, or 15 bytes")
    if len(payload) >= 11 and payload[10] & 0xC0:
        raise ValueError("reserved telemetry bits must be zero")
    packed = int.from_bytes(payload, "little")
    result = {}
    for name, offset, width, minimum, maximum in FIELDS:
        if offset + width > len(payload) * 8:
            break
        value = ((packed >> offset) & ((1 << width) - 1)) + minimum
        if not minimum <= value <= maximum:
            raise ValueError(f"invalid wire value for {name}")
        result[name] = value
    return result


BARCODE_BYTES = 4
BARCODE_RUNS_MAX = 29


def decode_barcode(payload):
    """4-byte <id>/barcode result: frame 0-15, letter 16-20, runs 21-25."""
    if len(payload) != BARCODE_BYTES:
        raise ValueError("barcode result must be 4 bytes")
    packed = int.from_bytes(payload, "little")
    letter = (packed >> 16) & 0x1F
    runs = (packed >> 21) & 0x1F
    if packed >> 26:
        raise ValueError("reserved barcode bits must be zero")
    if letter > 26 or runs > BARCODE_RUNS_MAX:
        raise ValueError("invalid barcode wire value")
    return {
        "frame": packed & 0xFFFF,
        "status": "valid" if letter else "invalid",
        "letter": chr(ord("A") + letter - 1) if letter else None,
        "runs": runs,
    }


def encode_barcode(frame, letter, runs):
    """Test/simulation helper; letter is "A".."Z" or None for invalid."""
    if type(frame) is not int or frame < 0:
        raise ValueError("frame must be a non-negative integer")
    if type(runs) is not int or not 0 <= runs <= BARCODE_RUNS_MAX:
        raise ValueError(f"runs must be an integer in 0..{BARCODE_RUNS_MAX}")
    if letter is None:
        code = 0
    elif isinstance(letter, str) and len(letter) == 1 and "A" <= letter <= "Z":
        code = ord(letter) - ord("A") + 1
    else:
        raise ValueError("letter must be one uppercase A-Z or None")
    return ((frame & 0xFFFF) | code << 16 | runs << 21).to_bytes(BARCODE_BYTES, "little")


def encode_telemetry(values):
    unknown = set(values) - {field[0] for field in FIELDS}
    if unknown:
        raise ValueError(f"unknown fields: {sorted(unknown)}")
    packed = 0
    for name, offset, width, minimum, maximum in FIELDS:
        value = values.get(name, 0)
        if type(value) is not int or not minimum <= value <= maximum:
            raise ValueError(f"{name} must be an integer in {minimum}..{maximum}")
        packed |= (value - minimum) << offset
    return packed.to_bytes(15, "little")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    decode = modes.add_parser("decode", help="decode one telemetry hex string")
    decode.add_argument("hex")
    modes.add_parser("stream", help="decode telemetry hex, one line per packet on stdin")
    command = modes.add_parser("command", help="write one raw command byte to stdout")
    command.add_argument("code", choices=COMMANDS)
    mapping = modes.add_parser("mapping", help="write a raw three-byte barcode map")
    for action in ("left", "right", "straight", "uturn"):
        mapping.add_argument(f"--{action}", required=True, metavar="LETTER")
    barcode = modes.add_parser("barcode", help="decode one barcode result hex string")
    barcode.add_argument("hex")
    modes.add_parser("barcode-stream",
                     help="decode barcode result hex, one line per packet on stdin")
    encode = modes.add_parser("encode", help="write raw telemetry from a JSON object")
    encode.add_argument("json")
    args = parser.parse_args()
    try:
        if args.mode == "decode":
            print(json.dumps(decode_telemetry(bytes.fromhex(args.hex))))
        elif args.mode == "stream":
            for line in sys.stdin:
                try:
                    print(json.dumps(decode_telemetry(bytes.fromhex(line.strip()))), flush=True)
                except ValueError as error:
                    print(f"Rejected packet: {error}", file=sys.stderr)
        elif args.mode == "barcode":
            print(json.dumps(decode_barcode(bytes.fromhex(args.hex))))
        elif args.mode == "barcode-stream":
            for line in sys.stdin:
                try:
                    print(json.dumps(decode_barcode(bytes.fromhex(line.strip()))), flush=True)
                except ValueError as error:
                    print(f"Rejected packet: {error}", file=sys.stderr)
        elif args.mode == "command":
            sys.stdout.buffer.write(bytes([COMMANDS[args.code]]))
        elif args.mode == "mapping":
            sys.stdout.buffer.write(encode_mapping((args.left, args.right, args.straight, args.uturn)))
        else:
            values = json.loads(args.json)
            if not isinstance(values, dict):
                raise ValueError("expected a JSON object")
            sys.stdout.buffer.write(encode_telemetry(values))
    except (ValueError, TypeError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
