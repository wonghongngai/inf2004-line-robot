#!/usr/bin/env python3
"""Guided robot tests over MQTT (#15, #38, #72): S1-U12 to S1-U15 and S1-U17.

Runs the broker side of each test against the live integrated robot and
writes one evidence log (everything printed is also saved). Steps that only a
person can see (wheels, console lines, a junction on the track) are prompted.

    python3 tools/robot_mqtt_test.py --broker 172.20.10.2 --client-id group17-robot

Needs mosquitto_sub / mosquitto_pub on PATH and the robot connected to the
same broker. The dashboard can stay open: both subscribe at once. Tests (see
testing/TEST_PLAN.md):

  S1-U12  telemetry for --minutes: ~1 packet/s, 19 bytes (#41), all valid, no gap > 3 s
  S1-U14  PAUSE / RESUME / STATUS each --repeats times; telemetry must show the
          effect within --timeout (operator confirms the wheels)
  S1-U13  stop the broker for --outage s, then restart it; then the same with
          the hotspot: telemetry back within 10 s of each restart, wheels
          unaffected (operator). Run it on the laptop that hosts the broker:
          the start and end of each outage are detected by polling the
          broker's TCP port, so no Enter press is timed
  S1-U20  runs inside S1-U13's hotspot case (#49): a PAUSE published while the
          robot is off the network must never be applied after it reconnects;
          then PAUSE x2 / RESUME x2 give one state change each and START while
          armed is ignored. Recorded as its own result
  S1-U15  maps with duplicate letters and a reserved bit are rejected
          (map flag stays 0 after a reboot), a valid map is applied (flag 1),
          then the robot turns as mapped at a junction (operator)
  S1-U17  --flood STATUS commands in about 1 s while following: telemetry keeps
          coming (no gap > 3 s), the robot keeps following, PAUSE still works
          straight after, and the console shows no new late control steps

The firmware side of S1-U11, S1-U13 and S1-U17 is in the serial console:
'wifi: link up ... t=<ms>' and '[S1:MQTT] connected ... t=<ms>' (ms since
boot), '[S1:CMD] commands rx=.. queued=.. invalid=.. dropped=..' and
'[S2:MOTION] ctrl late=<late>/<steps> gap_max=.. loop_max=.. ms'. Capture it
alongside this log.
"""
import argparse
import queue
import shutil
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mqtt_binary import (COMMANDS, TELEMETRY_BYTES, decode_telemetry,  # noqa: E402
                         encode_mapping)

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
        self.args = args
        self.packets = queue.Queue()
        self.process = None
        self.restart()

    def restart(self):
        """(Re)subscribe, e.g. after the broker or the hotspot came back."""
        if self.process is not None:
            self.process.terminate()
        args = self.args
        self.process = subprocess.Popen(
            ["mosquitto_sub", "-h", args.broker, "-p", str(args.port), "-q", "1",
             "-t", f"{args.client_id}/telemetry", "-F", "%x"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        threading.Thread(target=self._read, args=(self.process,), daemon=True).start()

    def _read(self, process):
        for line in process.stdout:
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


def publish(args, topic, payload, host=None):
    subprocess.run(["mosquitto_pub", "-h", host or args.broker, "-p", str(args.port), "-q", "1",
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


def wait_with_countdown(log, seconds, what, watch=None):
    """Count down; with watch, poll it every 0.25 s and stop early if it turns
    true. Returns the time.monotonic() it turned true, or None."""
    log(f"waiting {seconds:.0f} s ({what})")
    end = time.monotonic() + seconds
    early = None
    while time.monotonic() < end:
        left = end - time.monotonic()
        print(f"\r  {left:4.0f} s left ", end="", flush=True)
        if watch is not None and watch():
            early = time.monotonic()
            break
        time.sleep(min(0.25 if watch else 1.0, max(left, 0.0)))
    print("\r" + " " * 20 + "\r", end="", flush=True)
    return early


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
    wrong_size = sum(1 for size in sizes if size != TELEMETRY_BYTES)
    log(f"packets={len(stamps)} rate={rate:.2f}/s max_gap={max_gap:.2f}s "
        f"invalid={len(invalid)} not_{TELEMETRY_BYTES}_bytes={wrong_size}")
    for error in invalid[:5]:
        log(f"  invalid packet: {error}")
    ok = stamps and 0.8 <= rate <= 1.5 and max_gap <= 3.0 and not invalid and not wrong_size
    log(f"S1-U12 {'PASS' if ok else 'FAIL'} (expect ~1/s, all {TELEMETRY_BYTES} bytes and valid, no gap > 3 s)")
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


def broker_reachable(args):
    """True if the broker's TCP port accepts a connection right now."""
    try:
        with socket.create_connection((args.broker, args.port), timeout=0.3):
            return True
    except OSError:
        return False


def poll_until(up, want, limit):
    """Poll every 0.1 s until up() == want; return that moment
    (time.monotonic()) or None after limit seconds."""
    deadline = time.monotonic() + limit
    while time.monotonic() < deadline:
        if up() == want:
            return time.monotonic()
        time.sleep(0.1)
    return None


def laptop_ssid():
    """SSID this laptop's Wi-Fi is connected to (Windows netsh), or None when
    disconnected; raises OSError where netsh is not available."""
    out = subprocess.run(["netsh", "wlan", "show", "interfaces"], capture_output=True,
                         timeout=5).stdout.decode("utf-8", errors="replace")  # netsh writes the SSID as UTF-8
    state = ssid = None
    for line in out.splitlines():
        key, sep, value = line.partition(":")
        if not sep:
            continue
        key = key.strip()
        if key == "State":
            state = value.strip()
        elif key == "SSID":
            ssid = value.strip()
    return ssid if state == "connected" else None


def outage_case(args, log, telemetry, name, stop, start, during=None, up=None, what=None):
    """One S1-U13 case: telemetry flowing, outage, time until it is back.

    The outage start and end are detected by polling up() (default: the
    broker's TCP port), not by the operator pressing Enter. Recovery is
    timed from the moment up() is true again."""
    if up is None:
        up = lambda: broker_reachable(args)  # noqa: E731
        what = what or f"broker {args.broker}:{args.port}"
    telemetry.drain()
    if telemetry.wait_for(lambda d: True, 5.0) is None:
        log(f"{name}: FAIL, no telemetry before the outage (check the setup)")
        return False
    print(f"  >> {stop} (detected automatically)", flush=True)
    log(f"OPERATOR asked: {stop}")
    down = poll_until(up, False, args.react_wait)
    if down is None:
        log(f"{name}: FAIL, {what} still up after {args.react_wait:.0f} s; "
            f"the outage never started")
        return False
    log(f"{name}: {what} down (outage started)")
    # Keep polling during the countdown: if the broker or hotspot comes back
    # early (run 3, 22 Nov: the laptop rejoined 13 s before the prompt), time
    # the recovery from that moment, not from the end of the countdown.
    reachable = up

    early = None
    if during is not None:
        # Give the robot a few seconds to lose the link before acting.
        early = wait_with_countdown(log, min(5.0, args.outage),
                                    f"{name} outage, before the offline step", watch=reachable)
        if early is None:
            during()
            early = wait_with_countdown(log, max(args.outage - 5.0, 0.0), f"{name} outage",
                                        watch=reachable)
        else:
            log(f"{name}: back before the offline step; it was skipped")
    else:
        early = wait_with_countdown(log, args.outage, f"{name} outage", watch=reachable)
    if early is not None:
        back = early
        log(f"{name}: WARN back after only {back - down:.1f} s, before the {args.outage:.0f} s "
            f"outage ended (restored too early?); timing from then")
    else:
        print(f"  >> {start} (detected automatically)", flush=True)
        log(f"OPERATOR asked: {start}")
        back = poll_until(up, True, args.react_wait)
        if back is None:
            log(f"{name}: FAIL, {what} not up again within {args.react_wait:.0f} s "
                f"of asking; nothing to time")
            return False
    log(f"{name}: {what} up again after {back - down:.1f} s off; timing telemetry from here")
    # The subscriber lost the broker too; resubscribe until telemetry arrives.
    got = None
    deadline = back + args.recover_wait
    while got is None and time.monotonic() < deadline:
        telemetry.restart()
        telemetry.drain()
        got = telemetry.wait_for(lambda d: True, min(2.0, max(deadline - time.monotonic(), 0.1)))
    if got is None:
        log(f"{name}: no telemetry within {args.recover_wait:.0f} s of {what} being back")
        return False
    recovered = got[0] - back
    log(f"{name}: telemetry back {recovered:.1f} s after {what} was back "
        f"({describe(got[1])})")
    return recovered <= 10.0


def test_s1_u13(args, log, telemetry):
    log(f"== S1-U13 reconnect: broker and hotspot off for {args.outage:.0f} s each ==")
    pause_for(log, "Robot STARTed and following (or wheels raised and turning), serial console captured.")
    broker = outage_case(args, log, telemetry, "broker",
                         "Stop the broker now (Ctrl-C the mosquitto window).",
                         "Start the broker again.")
    offline_sent = []

    def offline_pause():
        # The laptop is off the hotspot too, so reach its broker over loopback.
        try:
            command_at(args, "pause", args.local_broker)
            offline_sent.append(time.monotonic())
            log(f"S1-U20: PAUSE published via {args.local_broker} while the robot is offline")
        except (subprocess.SubprocessError, OSError) as error:
            log(f"S1-U20: could not publish the offline PAUSE ({error})")

    # Hotspot: Windows keeps the laptop's address for a while after it leaves
    # the hotspot (run 4, 22 Nov: 2 min), so the broker keeps answering over
    # loopback. Use the laptop's own Wi-Fi state instead: up = connected to
    # the hotspot it is on now, and the broker answers.
    try:
        hotspot_ssid = laptop_ssid()
    except (OSError, subprocess.SubprocessError):
        hotspot_ssid = None
    if hotspot_ssid:
        log(f"hotspot: watching this laptop's Wi-Fi (SSID {hotspot_ssid!r}) for the outage")

        def hotspot_up():
            try:
                return laptop_ssid() == hotspot_ssid and broker_reachable(args)
            except (OSError, subprocess.SubprocessError):
                return False
        hotspot_what = f"laptop on {hotspot_ssid!r}"
    else:
        log("hotspot: WARN cannot read this laptop's Wi-Fi state (netsh); falling back to "
            "the broker port, which can miss the outage")
        hotspot_up = hotspot_what = None
    hotspot = outage_case(args, log, telemetry, "hotspot",
                          "Turn the hotspot (AP) off now.",
                          "Turn the hotspot back on.", during=offline_pause,
                          up=hotspot_up, what=hotspot_what)
    args.extra_results["S1-U20"] = test_s1_u20(args, log, telemetry, hotspot, bool(offline_sent))
    log("Firmware times: compare the 'wifi: link up ... t=' and '[S1:MQTT] connected ... t=' "
        "lines in the console with the moment each outage ended.")
    wheels = ask(log, "Did the wheels keep turning (or the robot keep following) through both outages?")
    ok = broker and hotspot and wheels
    log(f"S1-U13 {'PASS' if ok else 'FAIL'} (expect telemetry back within 10 s of each restart; "
        f"Wi-Fi loss must not stop driving)")
    return ok


def command_at(args, name, host):
    publish(args, "command", bytes([COMMANDS[name]]), host=host)


def states_for(telemetry, seconds):
    """Telemetry states seen in the next `seconds` (valid packets only)."""
    end = time.monotonic() + seconds
    states = []
    while time.monotonic() < end:
        try:
            _, _, data, error = telemetry.packets.get(timeout=0.5)
        except queue.Empty:
            continue
        if error is None:
            states.append(data["state"])
    return states


DRIVING = (1, 2, 4, 5)


def test_s1_u20(args, log, telemetry, reconnected, offline_sent):
    """#49: commands sent while offline, then duplicates, after a reconnect."""
    log("== S1-U20 commands across a reconnect (#49) ==")
    if not offline_sent:
        log("S1-U20 FAIL: the offline PAUSE was never published (nothing to check)")
        return False
    if not reconnected:
        log("S1-U20 FAIL (inconclusive): no telemetry after the reconnect, so the "
            "robot's state cannot be read")
        return False

    # 1. The PAUSE sent while offline must not arrive late.
    states = states_for(telemetry, args.stale_watch)
    paused = [s for s in states if s == 0]
    log(f"after the reconnect: {len(states)} packets in {args.stale_watch:.0f} s, "
        f"states={sorted({STATES[s] for s in states}) or 'none'}")
    stale_ok = bool(states) and not paused
    log(f"offline PAUSE {'never applied' if stale_ok else 'APPLIED LATE or no telemetry'}")

    # 2. PAUSE twice: paused once, and stays paused.
    telemetry.drain()
    command(args, "pause")
    command(args, "pause")
    command(args, "status")
    got = telemetry.wait_for(lambda d: d["state"] == 0, args.timeout)
    held = got is not None and all(s == 0 for s in states_for(telemetry, 2.0))
    log(f"PAUSE x2 -> {'IDLE and stayed IDLE' if held else 'not paused, or did not stay paused'}")

    # 3. RESUME twice: following again.
    telemetry.drain()
    command(args, "resume")
    command(args, "resume")
    command(args, "status")
    got = telemetry.wait_for(lambda d: d["state"] in DRIVING, args.timeout)
    resumed = got is not None and all(s in DRIVING for s in states_for(telemetry, 2.0))
    log(f"RESUME x2 -> {'driving and stayed driving' if resumed else 'not resumed, or stopped again'}")

    # 4. START while armed: ignored, keeps driving.
    telemetry.drain()
    command(args, "start")
    command(args, "status")
    states = states_for(telemetry, 2.0)
    start_kept = bool(states) and all(s in DRIVING for s in states)
    log(f"START while armed -> {'still driving' if start_kept else 'state changed or no telemetry'}")
    console = ask(log, "Did the console print 'START ignored: already started' once, and no "
                       "PAUSE being applied right after the reconnect?")
    ok = stale_ok and held and resumed and start_kept and console
    log(f"S1-U20 {'PASS' if ok else 'FAIL'} (expect the offline PAUSE never applied, duplicates "
        f"harmless, START while armed ignored)")
    return ok


def test_s1_u17(args, log, telemetry):
    log(f"== S1-U17 command flood: {args.flood} STATUS commands in about 1 s ==")
    pause_for(log, "Robot STARTed and following (or wheels raised and turning), serial console captured.")
    telemetry.drain()
    before = telemetry.wait_for(lambda d: True, 5.0)
    if before is None:
        log("S1-U17 FAIL: no telemetry before the flood (check the setup)")
        return False
    log(f"before: {describe(before[1])}")
    late_before = input("  >> Type the last 'ctrl late=X/Y' X value from the console: ").strip()
    log(f"OPERATOR: ctrl late before = {late_before}")

    telemetry.drain()
    started = time.monotonic()
    # One client, --repeat: a new mosquitto_pub per message would be too slow.
    subprocess.run(["mosquitto_pub", "-h", args.broker, "-p", str(args.port), "-q", "1",
                    "-t", f"{args.client_id}/command", "-s",
                    "--repeat", str(args.flood), "--repeat-delay", f"{1.0 / args.flood:.4f}"],
                   input=bytes([COMMANDS["status"]]), check=True, timeout=30)
    sent_s = time.monotonic() - started
    log(f"sent {args.flood} STATUS in {sent_s:.2f} s")

    # Telemetry during the flood and 5 s after it: no gap > 3 s, still following.
    end = time.monotonic() + 5.0
    stamps, states = [started], []
    while time.monotonic() < end:
        try:
            stamp, _, data, error = telemetry.packets.get(timeout=0.5)
        except queue.Empty:
            continue
        if error is None:
            stamps.append(stamp)
            states.append(data["state"])
    stamps.append(time.monotonic())
    max_gap = max(b - a for a, b in zip(stamps, stamps[1:]))
    stopped = [STATES[s] for s in states if s not in (1, 2, 4, 5)]
    log(f"telemetry during/after: packets={len(stamps) - 2} max_gap={max_gap:.2f}s "
        f"not-driving states={sorted(set(stopped)) or 'none'}")

    telemetry.drain()
    sent = time.monotonic()
    command(args, "pause")
    command(args, "status")
    paused = telemetry.wait_for(lambda d: d["state"] == 0, args.timeout)
    log(f"PAUSE after the flood -> {'IDLE after %.0f ms' % ((paused[0] - sent) * 1000) if paused else 'not paused'}")
    command(args, "resume")

    late_after = input("  >> Type the 'ctrl late=X/Y' X value from the console now: ").strip()
    log(f"OPERATOR: ctrl late after = {late_after}")
    counted = ask(log, f"Did a '[S1:CMD] commands rx=..' line appear, with every STATUS counted "
                       f"(rx = queued + invalid + dropped)?")
    no_late = late_before != "" and late_before == late_after
    ok = max_gap <= 3.0 and not stopped and paused is not None and no_late and counted
    log(f"S1-U17 {'PASS' if ok else 'FAIL'} (expect no telemetry gap > 3 s, still following, "
        f"PAUSE works, ctrl late unchanged, every command counted)")
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
    parser.add_argument("--outage", type=float, default=30.0, help="S1-U13 seconds off")
    parser.add_argument("--recover-wait", type=float, default=30.0,
                        help="S1-U13 seconds to wait for telemetry after the broker is back")
    parser.add_argument("--react-wait", type=float, default=180.0,
                        help="S1-U13 seconds to wait for the operator to stop or restore "
                             "the broker or hotspot")
    parser.add_argument("--local-broker", default="127.0.0.1",
                        help="S1-U20 address of the same broker that still works with the "
                             "hotspot off (the broker must run on this laptop)")
    parser.add_argument("--stale-watch", type=float, default=10.0,
                        help="S1-U20 seconds to watch for a late PAUSE after the reconnect")
    parser.add_argument("--flood", type=int, default=100, help="S1-U17 commands in about 1 s")
    parser.add_argument("--map", default="FAZW",
                        help="valid map for S1-U15 as LRSU letters (default FAZW: A turns right)")
    parser.add_argument("--only", choices=TESTS, action="append",
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
    args.extra_results = {}   # results a test records for another ID (S1-U13 -> S1-U20)
    try:
        tests = args.only or list(TESTS)
        for name, test in TESTS.items():
            if name in tests:
                results[name] = test(args, log, telemetry)
                results.update(args.extra_results)
                args.extra_results.clear()
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


TESTS = {"S1-U12": test_s1_u12, "S1-U13": test_s1_u13, "S1-U14": test_s1_u14,
         "S1-U15": test_s1_u15, "S1-U17": test_s1_u17}


if __name__ == "__main__":
    sys.exit(main())
