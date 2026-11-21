#!/usr/bin/env python3
"""Guided robot tests for #72 / #38 over MQTT: S1-U12, S1-U14 and S1-U15.

Runs the broker side of each test against the live integrated robot and
writes one evidence log (everything printed is also saved). Steps that only a
person can see (wheels, console lines, a junction on the track) are prompted.

    python3 tools/robot_mqtt_test.py --broker 172.20.10.2 --client-id group17-robot

Needs mosquitto_sub / mosquitto_pub on PATH and the robot connected to the
same broker. The dashboard can stay open: both subscribe at once. Tests (see
testing/TEST_PLAN.md):

  S1-U12  telemetry for --minutes: ~1 packet/s, 15 bytes, all valid, no gap > 3 s
  S1-U14  PAUSE / RESUME / STATUS each --repeats times; telemetry must show the
          effect within --timeout (operator confirms the wheels)
  S1-U15  maps with duplicate letters and a reserved bit are rejected
          (map flag stays 0 after a reboot), a valid map is applied (flag 1),
          then the robot turns as mapped at a junction (operator)
"""
import argparse
import queue
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mqtt_binary import COMMANDS, decode_telemetry, encode_mapping  # noqa: E402

STATES = ["IDLE", "FOLLOWING", "TURNING", "SCANNING", "AVOIDING", "RECOVERING",
          "FINISHED", "FAULT"]
DIRECTIONS = ["none", "LEFT", "RIGHT", "STRAIGHT", "U-TURN"]
ROOT = Path(__file__).resolve().parents[1]


class Log:
    """Print and save every line, with a wall-clock prefix."""

    def __init__(self, path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self.file = open(path, "w", encoding="utf-8")

    def __call__(self, text=""):
        line = f"{datetime.now():%H:%M:%S.%f}"[:-3] + f"  {text}" if text else ""
        print(line, flush=True)
        self.file.write(line + "\n")
        self.file.flush()

    def close(self):
        self.file.close()


class Telemetry:
    """Background mosquitto_sub on the telemetry topic; packets go to a queue."""

    def __init__(self, args):
        self.packets = queue.Queue()
        self.process = subprocess.Popen(
            ["mosquitto_sub", "-h", args.broker, "-p", str(args.port), "-q", "1",
             "-t", f"{args.client_id}/telemetry", "-F", "%x"],
            stdout=subprocess.PIPE, text=True)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.process.stdout:
            stamp = time.monotonic()
            hex_payload = line.strip()
            try:
                payload = bytes.fromhex(hex_payload)
                self.packets.put((stamp, len(payload), decode_telemetry(payload), None))
            except ValueError as error:
                self.packets.put((stamp, len(hex_payload) // 2, None, str(error)))

    def drain(self):
        while not self.packets.empty():
            self.packets.get_nowait()

    def wait_for(self, predicate, timeout):
        """First valid packet matching predicate within timeout, or None."""
        deadline = time.monotonic() + timeout
        while True:
            left = deadline - time.monotonic()
            if left <= 0:
                return None
            try:
                stamp, _, data, error = self.packets.get(timeout=left)
            except queue.Empty:
                return None
            if error is None and predicate(data):
                return stamp, data

    def close(self):
        self.process.terminate()


def publish(args, topic, payload):
    subprocess.run(["mosquitto_pub", "-h", args.broker, "-p", str(args.port), "-q", "1",
                    "-t", f"{args.client_id}/{topic}", "-s"],
                   input=payload, check=True, timeout=5)


def command(args, name):
    publish(args, "command", bytes([COMMANDS[name]]))


def ask(log, question):
    """Operator yes/no answer, recorded in the log."""
    while True:
        answer = input(f"  >> {question} [y/n] ").strip().lower()
        if answer in ("y", "n"):
            log(f"OPERATOR: {question} -> {'yes' if answer == 'y' else 'no'}")
            return answer == "y"


def pause_for(log, instruction):
    input(f"  >> {instruction} Press Enter when done. ")
    log(f"OPERATOR: done: {instruction}")


def describe(data):
    return (f"state={STATES[data['state']]} speed={data['speed_cms']} "
            f"line={data['line_sensors']:03b} letter={data.get('barcode_letter', '-')} "
            f"map={data.get('map_received', '-')}")


def test_s1_u12(args, log, telemetry):
    log(f"== S1-U12 telemetry for {args.minutes} min ==")
    telemetry.drain()
    end = time.monotonic() + args.minutes * 60.0
    stamps, sizes, invalid = [], [], []
    while time.monotonic() < end:
        try:
            stamp, size, data, error = telemetry.packets.get(timeout=1.0)
        except queue.Empty:
            continue
        stamps.append(stamp)
        sizes.append(size)
        if error is not None:
            invalid.append(error)
    duration = args.minutes * 60.0
    gaps = [b - a for a, b in zip(stamps, stamps[1:])]
    max_gap = max(gaps) if gaps else float("inf")
    rate = len(stamps) / duration if duration else 0.0
    wrong_size = sum(1 for size in sizes if size != 15)
    log(f"packets={len(stamps)} rate={rate:.2f}/s max_gap={max_gap:.2f}s "
        f"invalid={len(invalid)} not_15_bytes={wrong_size}")
    for error in invalid[:5]:
        log(f"  invalid packet: {error}")
    ok = stamps and 0.8 <= rate <= 1.5 and max_gap <= 3.0 and not invalid and not wrong_size
    log(f"S1-U12 {'PASS' if ok else 'FAIL'} (expect ~1/s, all 15 bytes and valid, no gap > 3 s)")
    return bool(ok)


def test_s1_u14(args, log, telemetry):
    log(f"== S1-U14 PAUSE / RESUME / STATUS x{args.repeats} ==")
    pause_for(log, "Wheels raised (or robot on the line), robot STARTed and following.")
    results = {"pause": 0, "resume": 0, "status": 0}
    for i in range(1, args.repeats + 1):
        telemetry.drain()
        sent = time.monotonic()
        command(args, "status")
        got = telemetry.wait_for(lambda d: True, args.timeout)
        ok = got is not None
        results["status"] += ok
        log(f"[{i}] STATUS -> {'packet after %.0f ms (%s)' % ((got[0] - sent) * 1000, describe(got[1])) if ok else 'no packet'}")

        telemetry.drain()
        sent = time.monotonic()
        command(args, "pause")
        command(args, "status")
        got = telemetry.wait_for(lambda d: d["state"] == 0, args.timeout)
        ok = got is not None
        results["pause"] += ok
        log(f"[{i}] PAUSE  -> {'IDLE after %.0f ms' % ((got[0] - sent) * 1000) if ok else 'not paused within %.1f s' % args.timeout}")
        time.sleep(0.5)

        telemetry.drain()
        sent = time.monotonic()
        command(args, "resume")
        command(args, "status")
        got = telemetry.wait_for(lambda d: d["state"] in (1, 2, 4, 5), args.timeout)
        ok = got is not None
        results["resume"] += ok
        log(f"[{i}] RESUME -> {'%s after %.0f ms' % (STATES[got[1]['state']], (got[0] - sent) * 1000) if ok else 'not resumed within %.1f s' % args.timeout}")
        time.sleep(0.5)
    log(f"broker side: STATUS {results['status']}/{args.repeats}, "
        f"PAUSE {results['pause']}/{args.repeats}, RESUME {results['resume']}/{args.repeats}")
    wheels = ask(log, f"Did the wheels stop at every PAUSE (within ~0.5 s) and restart at every RESUME?")
    ok = all(v == args.repeats for v in results.values()) and wheels
    log(f"S1-U14 {'PASS' if ok else 'FAIL'} (expect {args.repeats}/{args.repeats} each; PAUSE stops within 500 ms)")
    return ok


def map_flag(args, telemetry):
    telemetry.drain()
    command(args, "status")
    got = telemetry.wait_for(lambda d: "map_received" in d, args.timeout)
    return None if got is None else got[1]["map_received"]


def test_s1_u15(args, log, telemetry):
    log("== S1-U15 letter map over MQTT ==")
    pause_for(log, "Reboot the robot (no map), wait for Wi-Fi + MQTT, and keep the serial console open.")
    flag = map_flag(args, telemetry)
    log(f"map flag after reboot = {flag}")
    if flag != 0:
        log("S1-U15 FAIL: the flag must be 0 before any map is sent")
        return False
    bad = {"duplicate letters A/A/A/A": bytes.fromhex("000000"),
           "reserved bit set (A/B/C/Z + bit 20)": bytes.fromhex("20881c")}
    rejected = True
    for name, payload in bad.items():
        publish(args, "mapping", payload)
        time.sleep(0.5)
        flag = map_flag(args, telemetry)
        log(f"sent {name} ({payload.hex()}) -> map flag {flag}")
        rejected &= flag == 0
    console = ask(log, "Did the console print 'Buddy 1: invalid barcode map rejected' twice?")
    letters = [args.map[i] for i in range(4)]
    publish(args, "mapping", encode_mapping(letters))
    time.sleep(0.5)
    flag = map_flag(args, telemetry)
    log(f"sent valid map L={letters[0]} R={letters[1]} S={letters[2]} U={letters[3]} -> map flag {flag}")
    applied = flag == 1 and ask(log, "Did the console print '[ROBOT] MQTT map applied: ...' with those letters?")
    publish(args, "mapping", bytes.fromhex("000000"))   # an invalid map after a valid one
    log("sent the duplicate-letter map again: the valid map must be kept")
    drive = ask(log, f"Drive over barcode '{letters[1]}' then a junction: did the robot turn RIGHT "
                     f"(the old map kept, not changed by the bad one)?")
    ok = rejected and console and applied and drive
    log(f"S1-U15 {'PASS' if ok else 'FAIL'} (expect both invalid maps rejected, valid map applied and kept)")
    return ok


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--broker", required=True)
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--client-id", default="group17-robot")
    parser.add_argument("--minutes", type=float, default=5.0, help="S1-U12 duration")
    parser.add_argument("--repeats", type=int, default=10, help="S1-U14 repetitions")
    parser.add_argument("--timeout", type=float, default=1.5, help="seconds to see an effect")
    parser.add_argument("--map", default="FAZW",
                        help="valid map for S1-U15 as LRSU letters (default FAZW: A turns right)")
    parser.add_argument("--only", choices=("S1-U12", "S1-U14", "S1-U15"), action="append",
                        help="run only these tests (repeatable); default all")
    parser.add_argument("--out", type=Path, help="log path (default testing/integration/logs/...)")
    args = parser.parse_args()
    args.map = args.map.upper()
    if len(args.map) != 4 or len(set(args.map)) != 4 or not args.map.isalpha():
        parser.error("--map needs four different letters, e.g. FAZW")
    for tool in ("mosquitto_sub", "mosquitto_pub"):
        if shutil.which(tool) is None:
            parser.error(f"{tool} is not on PATH")
    out = args.out or ROOT / "testing/integration/logs" / f"{datetime.now():%Y-%m-%d_%H%M}_robot_mqtt_test.txt"
    log = Log(out)
    log(f"robot_mqtt_test.py broker={args.broker}:{args.port} client={args.client_id}")
    commit = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
                            capture_output=True, text=True).stdout.strip()
    log(f"repo commit {commit or 'unknown'} (record the firmware's commit in TEST_LOG)")
    telemetry = Telemetry(args)
    results = {}
    try:
        tests = args.only or ["S1-U12", "S1-U14", "S1-U15"]
        for name, test in (("S1-U12", test_s1_u12), ("S1-U14", test_s1_u14), ("S1-U15", test_s1_u15)):
            if name in tests:
                results[name] = test(args, log, telemetry)
                log()
    except KeyboardInterrupt:
        log("interrupted")
    finally:
        telemetry.close()
        summary = ", ".join(f"{k} {'PASS' if v else 'FAIL'}" for k, v in results.items())
        log(f"SUMMARY: {summary or 'no test completed'}")
        log(f"log saved to {out}")
        log.close()
    return 0 if results and all(results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
