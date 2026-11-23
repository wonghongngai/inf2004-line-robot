Barcode result events
=====================

Subscribe to `<MQTT_CLIENT_ID>/barcode` using the configured broker. The
topic can be overridden with `MQTT_BARCODE_TOPIC`. Each payload is exactly
4 binary bytes (protocol v1, like telemetry). Bit 0 is the least significant
bit of byte 0, and multi-bit fields are little-endian:

| Field | Range | Bits | Bit offset |
| --- | --- | ---: | ---: |
| Frame number | 0–65535, low 16 bits of the per-boot count (wraps) | 16 | 0 |
| Letter | 0 rejected capture, 1–26 = A–Z (same as telemetry bits 80–84) | 5 | 16 |
| Runs captured | 0–29 bars and gaps (29 = a full Code 39 character) | 5 | 21 |
| Reserved | MUST be zero | 6 | 26 |

Examples: frame 1, Z, 29 runs = `01 00 ba 03`; frame 65535, rejected,
23 runs = `ff ff e0 02`. Receivers reject any other length, a reserved bit,
letter 27–31 or runs 30–31. The frame number matches the low 16 bits of the
`[BARCODE FRAME] id=` console line.

Decode them with `tools/mqtt_binary.py` (Mosquitto hex output, one packet per
line):

```sh
mosquitto_sub -h BROKER -t 'CLIENT_ID/barcode' -F '%x' | python3 tools/mqtt_binary.py barcode-stream
python3 tools/mqtt_binary.py barcode 0100ba03
# {"frame": 1, "status": "valid", "letter": "Z", "runs": 29}
```

The C codec is `s1_barcode_pack()` / `s1_barcode_unpack()` in
`communication_protocol.c`. If the layout changes, update it, the Python
decoder, both host tests and this table together.

The S3 sampling task (`line_service.c`) enqueues an event when capture emits
BARCODE or INVALID, including timeout/rejected partial captures, by calling
`subsystem1_publish_barcode()`. Merely seeing black does not publish a
result. No result is emitted for a capture discarded by pause, reset, timing
fault, or junction classification before decode completes.

The radio task (`mqtt_manager.c`) publishes with QoS 1 and retain=false. It
keeps the pending event until the broker's PUBACK; a failed or timed-out
publish is retried, and an event in flight when the connection drops is sent
again after reconnecting. QoS 1 can therefore duplicate messages; frame IDs
are per boot, not globally unique.

The dedicated queue holds four waiting events plus the radio's pending event.
Events wait for connectivity. A full queue rejects new events and logs
`[S3:BARCODE] ERR MQTT enqueue=<negative error> frame=<id> (queue full or
invalid)`; a successful enqueue is silent. Queued events are lost on reboot.
Each broker acknowledgement logs `[S1:MQTT] barcode acked frame=<id>`; it
does not confirm that a subscriber processed the message. Builds without
MQTT skip the queue silently. The dashboard (`tools/mqtt_dashboard.py`)
subscribes to this topic too (`--barcode-topic` overrides it).

Since S3 numbers only the results it hands to S1, a subscriber should see
every frame number exactly once, each one more than the last (plus QoS 1
repeats). A skipped number means an event was lost on the robot.

The original #3 design used JSON text. It was changed to this binary packet
so that every topic follows protocol v1.

Host tests (build flags live in `testing/run_host_tests.sh`):

```sh
sh testing/run_host_tests.sh s1/   # queue, C codec, Python decoder
```

Broker delivery is tested on the robot as S1-U16 (#16):

```sh
python3 tools/robot_mqtt_test.py --broker BROKER --only S1-U16   # 5 barcodes
```

It counts distinct frames, flags gaps, reordering and undecodable packets,
and with `--letters` checks the decoded letters in order.
