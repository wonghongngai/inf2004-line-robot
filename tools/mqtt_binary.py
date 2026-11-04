#!/usr/bin/env python3
"""Protocol v1 Mac helper. No external packages. Hex decode or raw command output."""
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
)
COMMANDS = {"status": 0, "pause": 1, "resume": 2, "reset-stats": 3}


def decode_telemetry(payload):
    if len(payload) != 10:
        raise ValueError("telemetry must be exactly 10 bytes")
    packed = int.from_bytes(payload, "little")
    result = {}
    for name, offset, width, minimum, maximum in FIELDS:
        value = ((packed >> offset) & ((1 << width) - 1)) + minimum
        if not minimum <= value <= maximum:
            raise ValueError(f"invalid wire value for {name}")
        result[name] = value
    return result


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
    return packed.to_bytes(10, "little")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest="mode", required=True)
    decode = modes.add_parser("decode", help="decode one telemetry hex string")
    decode.add_argument("hex")
    modes.add_parser("stream", help="decode telemetry hex, one line per packet on stdin")
    command = modes.add_parser("command", help="write one raw command byte to stdout")
    command.add_argument("code", choices=COMMANDS)
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
        elif args.mode == "command":
            sys.stdout.buffer.write(bytes([COMMANDS[args.code]]))
        else:
            values = json.loads(args.json)
            if not isinstance(values, dict):
                raise ValueError("expected a JSON object")
            sys.stdout.buffer.write(encode_telemetry(values))
    except (ValueError, TypeError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
