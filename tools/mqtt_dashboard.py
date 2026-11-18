#!/usr/bin/env python3
"""Serve a local, read-only dashboard for robot MQTT telemetry."""
import argparse
import errno
import json
import shutil
import subprocess
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Lock, Thread
from time import monotonic, time
from urllib.parse import urlsplit

from mqtt_binary import decode_telemetry, encode_telemetry

ASSETS = Path(__file__).with_suffix("")
EXAMPLE = dict(state=1, speed_cms=28, speed_valid=1, distance_cm=140,
               line_sensors=2, barcode=1, barcode_letter=6, barcode_black=15, barcode_white=14, highest_hump_mm=22, obstacle_cm=34,
               scan_deg=90, obstacle_width_cm=16)


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
                telemetry = decode_telemetry(encode_telemetry({
                    **EXAMPLE, "distance_cm": (140 + step * 28) % 65536,
                    "line_sensors": (2, 2, 3, 2, 6)[step % 5],
                    "obstacle_cm": 34 + step % 12,
                    "scan_deg": 60 + step % 7 * 10,
                }))
                heartbeat, telemetry_at, heartbeat_at = True, now, now
            if heartbeat is not False and (heartbeat_at is None or now - heartbeat_at > 15):
                heartbeat = None
            return dict(
                demo=self.args.demo, client_id=self.args.client_id,
                telemetry_topic=self.args.telemetry_topic, status_topic=self.args.status_topic,
                telemetry=telemetry, telemetry_at=telemetry_at, heartbeat=heartbeat,
                stale=telemetry_at is not None and now - telemetry_at > 5,
                rejected=self.rejected,
            )


class DashboardHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, feed, **kwargs):
        self.feed = feed
        super().__init__(*args, directory=str(ASSETS), **kwargs)

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


def read_messages(process, feed):
    for line in process.stdout:
        feed.receive(line)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker", default="127.0.0.1", help="MQTT broker host")
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--client-id", default="group17-robot", help="robot topic prefix")
    parser.add_argument("--telemetry-topic", help="override <client-id>/telemetry")
    parser.add_argument("--status-topic", help="override <client-id>/status")
    parser.add_argument("--port", type=int, help="local HTTP port (default: 8000, or a free port if busy)")
    parser.add_argument("--demo", action="store_true", help="preview with simulated data")
    args = parser.parse_args()
    args.telemetry_topic = args.telemetry_topic or f"{args.client_id}/telemetry"
    args.status_topic = args.status_topic or f"{args.client_id}/status"
    if args.telemetry_topic == args.status_topic:
        parser.error("telemetry and status topics must differ")
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
            parser.error(f"Dashboard port {port} is in use. Try --port 0 for a free port.")
        print(f"Dashboard port {port} is in use. Choosing a free port.", flush=True)
        server = ThreadingHTTPServer(("127.0.0.1", 0), handler)
    with server:
        process = None
        try:
            if not args.demo:
                process = subprocess.Popen(
                    ["mosquitto_sub", "-h", args.broker, "-p", str(args.mqtt_port),
                     "-q", "1", "-t", args.telemetry_topic, "-t", args.status_topic,
                     "-F", "%t\t%x"],
                    stdout=subprocess.PIPE, text=True,
                )
                Thread(target=read_messages, args=(process, feed), daemon=True).start()
            print(f"Dashboard at http://127.0.0.1:{server.server_port}", flush=True)
            server.serve_forever()
        except KeyboardInterrupt:
            pass
        finally:
            if process is not None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()


if __name__ == "__main__":
    main()
