#!/usr/bin/env python3
"""Serve a local dashboard for robot MQTT telemetry and controls."""

import argparse
import errno
import json
import shutil
import subprocess
import sys
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock, Thread
from time import monotonic, sleep, time
from urllib.parse import parse_qs, urlsplit

from collections import deque

from mqtt_binary import (
    COMMANDS,
    decode_barcode,
    decode_telemetry,
    encode_barcode,
    encode_mapping,
    encode_telemetry,
)

ASSETS = Path(__file__).with_suffix("")
EXAMPLE = dict(
    state=1,
    speed_cms=28,
    speed_valid=1,
    distance_cm=140,
    map_received=1,
    line_sensors=2,
    barcode=1,
    barcode_letter=6,
    barcode_black=15,
    barcode_white=14,
    highest_hump_mm=22,
    obstacle_cm=34,
    scan_deg=90,
    obstacle_width_cm=16,
    left_ticks=16492,
    right_ticks=16562,
)
BARCODE_HISTORY = 20
DEMO_SCANS = ("A", "F", None, "Z", "W")  # None = a rejected capture


class Feed:
    def __init__(self, args):
        self.args = args
        self.lock = Lock()
        self.started = monotonic()
        self.telemetry = None
        self.telemetry_at = None
        self.heartbeat = None
        self.heartbeat_at = None
        self.rejected = 0
        self.barcodes = deque(maxlen=BARCODE_HISTORY)  # newest first
        self.barcode_total = 0

    def receive(self, line):
        try:
            topic, hex_payload = line.rstrip("\r\n").split("\t", 1)
            payload = bytes.fromhex(hex_payload)
            with self.lock:
                if topic == self.args.telemetry_topic:
                    self.telemetry = decode_telemetry(payload)
                    self.telemetry_at = time()
                elif topic == self.args.status_topic:
                    if payload not in (b"\x00", b"\x01"):
                        raise ValueError("invalid heartbeat")
                    self.heartbeat = bool(payload[0])
                    self.heartbeat_at = time()
                elif topic == self.args.barcode_topic:
                    scan = decode_barcode(payload)
                    latest = self.barcodes[0] if self.barcodes else None
                    # QoS 1 can redeliver the same event; keep one copy.
                    if latest is None or any(latest[k] != scan[k] for k in scan):
                        self.barcodes.appendleft({**scan, "received_at": time()})
                        self.barcode_total += 1
        except ValueError:
            with self.lock:
                self.rejected += 1

    def snapshot(self):
        with self.lock:
            now = time()
            telemetry = self.telemetry
            telemetry_at = self.telemetry_at
            heartbeat = self.heartbeat
            heartbeat_at = self.heartbeat_at
            if self.args.demo:
                step = int(monotonic() - self.started)
                telemetry = decode_telemetry(
                    encode_telemetry(
                        {
                            **EXAMPLE,
                            "distance_cm": (140 + step * 28) % 65536,
                            "line_sensors": (2, 2, 3, 2, 6)[step % 5],
                            "obstacle_cm": 34 + step % 12,
                            "scan_deg": 60 + step % 7 * 10,
                            # #41: ~118 ticks/cm (S2_ENC_*) at 28 cm/s, wrapping at 16 bits
                            "left_ticks": (16492 + step * 3298) % 65536,
                            "right_ticks": (16562 + step * 3312) % 65536,
                        }
                    )
                )
                heartbeat, telemetry_at, heartbeat_at = True, now, now
                count = step // 4 + 1  # one simulated scan every 4 s
                barcodes = [
                    {
                        **decode_barcode(
                            encode_barcode(
                                frame,
                                DEMO_SCANS[(frame - 1) % len(DEMO_SCANS)],
                                29 if DEMO_SCANS[(frame - 1) % len(DEMO_SCANS)] else 17,
                            )
                        ),
                        "received_at": now - (count - frame) * 4,
                    }
                    for frame in range(count, max(0, count - BARCODE_HISTORY), -1)
                ]
                barcode_total = count
            else:
                barcodes = list(self.barcodes)
                barcode_total = self.barcode_total
            if heartbeat is not False and (
                heartbeat_at is None or now - heartbeat_at > 15
            ):
                heartbeat = None
            return dict(
                demo=self.args.demo,
                client_id=self.args.client_id,
                telemetry_topic=self.args.telemetry_topic,
                status_topic=self.args.status_topic,
                barcode_topic=self.args.barcode_topic,
                barcodes=barcodes,
                barcode_total=barcode_total,
                telemetry=telemetry,
                telemetry_at=telemetry_at,
                heartbeat=heartbeat,
                stale=telemetry_at is not None and now - telemetry_at > 5,
                rejected=self.rejected,
            )


class DashboardHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, feed, **kwargs):
        self.feed = feed
        super().__init__(*args, directory=str(ASSETS), **kwargs)

    def end_headers(self):
        # Revalidate the page files so a browser never keeps an old dashboard.
        if not urlsplit(self.path).path.startswith("/api/"):
            self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def do_POST(self):
        prefix = "/api/command/"
        request = urlsplit(self.path)
        path = request.path
        command = path[len(prefix) :]
        args = self.feed.args
        mapping = path == "/api/mapping"
        if not mapping and not (
            path.startswith(prefix) and command in ("pause", "resume", "start")
        ):
            self.send_error(404)
            return
        if self.headers.get("X-Dashboard-Command") != "1":
            self.send_error(403)
            return
        if args.demo:
            self.send_error(409, "Controls require live MQTT mode")
            return
        if mapping:
            actions = ("left", "right", "straight", "uturn")
            try:
                fields = parse_qs(request.query, strict_parsing=True, max_num_fields=4)
                if set(fields) != set(actions):
                    raise ValueError
                payload = encode_mapping([fields[action][0] for action in actions])
            except ValueError:
                self.send_error(400, "Use four distinct letters A-Z")
                return
            topic = args.mapping_topic
        else:
            payload = bytes([COMMANDS[command]])
            topic = args.command_topic
        try:
            subprocess.run(
                [
                    "mosquitto_pub",
                    "-h",
                    args.broker,
                    "-p",
                    str(args.mqtt_port),
                    "-q",
                    "1",
                    "-t",
                    topic,
                    "-s",
                ],
                input=payload,
                check=True,
                timeout=3,
            )
        except (OSError, subprocess.SubprocessError):
            self.send_error(502, "MQTT publish failed")
            return
        self.send_response(204)
        self.end_headers()

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == "/api/telemetry":
            body = json.dumps(self.feed.snapshot()).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif path in ("/", "/index.html", "/dashboard.css", "/dashboard.js"):
            super().do_GET()
        else:
            self.send_error(404)


def subscribe(feed):
    args = feed.args
    command = [
        "mosquitto_sub",
        "-h",
        args.broker,
        "-p",
        str(args.mqtt_port),
        "-q",
        "1",
        "-t",
        args.telemetry_topic,
        "-t",
        args.status_topic,
        "-t",
        args.barcode_topic,
        "-F",
        "%t\t%x",
    ]
    while True:
        try:
            with subprocess.Popen(
                command, stdout=subprocess.PIPE, text=True
            ) as process:
                try:
                    for line in process.stdout:
                        feed.receive(line)
                finally:
                    if process.poll() is None:
                        process.terminate()
                        try:
                            process.wait(timeout=3)
                        except subprocess.TimeoutExpired:
                            process.kill()
            print(
                f"MQTT subscriber exited ({process.returncode}). Retrying in 2 seconds.",
                file=sys.stderr,
                flush=True,
            )
        except OSError as error:
            print(
                f"MQTT subscriber error: {error}. Retrying in 2 seconds.",
                file=sys.stderr,
                flush=True,
            )
        sleep(2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker", default="127.0.0.1", help="MQTT broker host")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument(
        "--client-id", default="group17-robot", help="robot topic prefix"
    )
    parser.add_argument("--telemetry-topic", help="override <client-id>/telemetry")
    parser.add_argument("--status-topic", help="override <client-id>/status")
    parser.add_argument("--mapping-topic", help="override <client-id>/mapping")
    parser.add_argument("--command-topic", help="override <client-id>/command")
    parser.add_argument("--barcode-topic", help="override <client-id>/barcode")
    parser.add_argument(
        "--port",
        type=int,
        help="local HTTP port (default: 8000, or a free port if busy)",
    )
    parser.add_argument(
        "--demo", action="store_true", help="preview with simulated data"
    )
    args = parser.parse_args()
    args.telemetry_topic = args.telemetry_topic or f"{args.client_id}/telemetry"
    args.status_topic = args.status_topic or f"{args.client_id}/status"
    args.mapping_topic = args.mapping_topic or f"{args.client_id}/mapping"
    args.command_topic = args.command_topic or f"{args.client_id}/command"
    args.barcode_topic = args.barcode_topic or f"{args.client_id}/barcode"
    if len({args.telemetry_topic, args.status_topic, args.barcode_topic}) != 3:
        parser.error("telemetry, status and barcode topics must differ")
    if not args.demo and shutil.which("mosquitto_sub") is None:
        parser.error("mosquitto_sub is required for live mode. Use --demo to preview.")

    feed = Feed(args)
    handler = partial(DashboardHandler, feed=feed)
    port = 8000 if args.port is None else args.port
    try:
        server = ThreadingHTTPServer(("127.0.0.1", port), handler)
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            raise
        if args.port is not None:
            parser.error(
                f"Dashboard port {port} is in use. Try --port 0 for a free port."
            )
        print(f"Dashboard port {port} is in use. Choosing a free port.", flush=True)
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    with server:
        print(f"Dashboard at http://127.0.0.1:{server.server_port}", flush=True)
        try:
            if args.demo:
                server.serve_forever()
            else:
                Thread(target=server.serve_forever, daemon=True).start()
                try:
                    subscribe(feed)
                finally:
                    server.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
