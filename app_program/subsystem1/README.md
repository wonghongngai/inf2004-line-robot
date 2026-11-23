# Buddy 1: binary Wi-Fi/MQTT communication (protocol v1)

The built-in micro T-Kernel Wi-Fi service is the only task allowed to call
CYW43/lwIP/MQTT. Other tasks pass packed messages through bounded RTOS queues.
No standalone SDK main, duplicate driver, or second network task is needed.

| File | Responsibility |
| --- | --- |
| `communication_protocol.h` | Ranges, state/fault/command constants, packet types |
| `communication_protocol.c` | Explicit bit packing, unpacking and range validation |
| `subsystem1.h` / `subsystem1.c` | Task-safe queue API and object creation |
| `communication_internal.h` | Private bridge to the Wi-Fi service |
| `mqtt_manager.c` | MQTT connection/recovery, binary RX/TX and heartbeat |
| `drv_buzzer.c` | GP22 piezo: a short 2 kHz beep each time Wi-Fi gets an IP (`WIFI_CONNECT_BEEP`, default 1); non-blocking, stopped by a one-shot alarm |
| `subsystem1_test.c` | On-target queue tests, enabled by `TEST=1` |

## Representation table

Telemetry uses 151 information bits in 19 bytes. Bit 86 is the letter-map flag (#38); bit 87 is reserved and must be zero. Older 10-byte packets end at bit 79 and omit the decoded letter and speed validity. Older 11-byte packets omit the barcode counts. Older 15-byte packets (before #41) omit the encoder counts.
Bit 0 is the least significant bit of byte 0. Each field's least significant
bit is stored at the listed offset. Multi-byte fields follow the same little
endian bit ordering. No native C bitfield ABI or struct cast is used.

| Field | Logical range / units | Bits | Bit offset |
| --- | --- | ---: | ---: |
| Robot state | 0–7 | 3 | 0 |
| Shared faults | 0–31, five independent flags | 5 | 3 |
| Battery | 0–100 percent | 7 | 8 |
| Signed speed | -200–200 cm/s, stored as speed + 200 | 9 | 15 |
| Distance travelled | 0–65535 cm | 16 | 24 |
| Line sensors | Three binary readings, 0–7 | 3 | 40 |
| Barcode | 0 none, 1 left, 2 right, 3 straight, 4 U-turn | 3 | 43 |
| Highest hump | 0–255 mm | 8 | 46 |
| Obstacle distance | 0–400 cm | 9 | 54 |
| Scan angle | 0–180 degrees | 8 | 63 |
| Obstacle width | 0–400 cm | 9 | 71 |
| Last decoded barcode letter | 0 none, 1–26 A–Z | 5 | 80 |
| Encoder speed valid | 0 unavailable, 1 valid | 1 | 85 |
| Letter map received (#38) | 0 waiting for map (straight at junctions), 1 a map has been applied | 1 | 86 |
| Barcode black bars | 0–65535, saturating session count | 16 | 88 |
| Barcode internal white gaps | 0–65535, saturating session count | 16 | 104 |
| Left encoder count (#41) | low 16 bits of S2's signed cumulative tick count, wraps | 16 | 120 |
| Right encoder count (#41) | low 16 bits of S2's signed cumulative tick count, wraps | 16 | 136 |

**Encoder counts (#41).** The robot sends the raw counts it used for distance
in the same step, truncated to their low 16 bits (two's complement). Read a
field as int16 (`tools/mqtt_binary.py`: `ticks_signed`), and take the
difference of two packets modulo 2^16 (`ticks_delta`). That is correct while a
wheel moves less than 32767 ticks between packets: about 2.8 m at the
interrupt encoder's ~118 ticks/cm (`S2_ENC_*_TICKS_PER_CM`). Packets are 1 s
apart and the top speed is 200 cm/s, so that never happens. The raw value
itself wraps about every 2.8 m of travel, which is why the PC takes
differences. 16 bits works for any S2 counter of 16 bits or more,
so it does not depend on the widths #52 will choose. Motor speed is the
existing signed speed field (average of the two wheels).

States: idle, following, turning, scanning, avoiding, recovering, finished,
fault (0–7 in that order). Fault bits 0–4: motor stall, encoder fault, IMU
fault, no echo, Wi-Fi loss. Sensors occupy bits left=0, centre=1, right=2;
agree whether 1 means black with Buddy 3 (proposed convention: 1 = black).
Fault flags describe the controller's snapshot, not automatic driver updates.

Barcode map (topic `<client-id>/mapping`, QoS 1): exactly 3 bytes. Four 5-bit
letters (A=0..Z=25) at bit offsets 0, 5, 10, 15 for LEFT, RIGHT, STRAIGHT,
U-TURN; bits 20–23 MUST be zero and the four letters must differ. Example
A/B/C/Z = `20 88 0c`. Invalid maps are rejected and the old map stays. The
controller applies a valid map with S3's `line_service_set_mapping()`.

Command codes: 0 request status, 1 pause, 2 resume, 3 reset statistics,
4 reverse, 5 start (#48: leave Ready and start navigating). Codes use bits
0–2 of one byte. Codes 6–7 are reserved and the
upper five bits must be zero. The MQTT layer accepts and queues reverse.
Reverse movement is pending in the controller.

Barcode result (topic `<client-id>/barcode`, QoS 1): exactly 4 bytes, one per
completed or rejected scan. Frame number bits 0–15 (low 16 bits, wraps),
letter bits 16–20 (0 rejected, 1–26 = A–Z), runs captured bits 21–25 (0–29);
bits 26–31 MUST be zero. Example frame 1 / Z / 29 runs = `01 00 ba 03`.
Queue, retry and logging details are in `testing/subsystem1/BARCODE_MQTT.md`.

A heartbeat uses bit 0 of one byte (0 offline / 1 online), with seven reserved
zero bits.
Byte padding is unavoidable for these single-message MQTT payloads.
No command is executed by Buddy 1: the controller must enforce mission rules,
including rejecting reset statistics during a run and deciding when pause/resume
are allowed. Navigation remains autonomous.

Out-of-range inputs are rejected without changing the packet. Unused wire codes
are invalid; there is currently no unavailable-value sentinel. Do not interpret
zero-initialized fields as measurements: the caller must supply actual data.
Where the associated fault is set, consumers must treat the affected sensor
measurement as unreliable. Battery monitoring requires real supporting hardware;
change the schema with the team if it is absent. Distances above range need a
future schema change, not silent wrapping. Highest hump is maintained by Buddy 4.

## Application API

```c
#include "subsystem1.h"

S1_TELEMETRY snapshot;
s1_telemetry_init(&snapshot); /* idle, no faults, logical speed zero */
/* Fill ALL required fields from a consistent controller snapshot. */
if (!s1_telemetry_set(&snapshot, S1_FIELD_SPEED_CMS, 50)) {
    /* Report out-of-range measurement. */
}
(void)s1_telemetry_set(&snapshot, S1_FIELD_HIGHEST_HUMP_MM, 12);
ER result = subsystem1_publish_telemetry(&snapshot);
S1_COMMAND command;
INT received = subsystem1_receive_command(&command, TMO_POL);
if (received == S1_COMMAND_BYTES) {
    /* Controller validates permission, then handles command.bytes[0]. */
}
```

Use a task-local packet. A setter performs read-modify-write across byte fields;
it is NOT safe to modify the same packet concurrently. Take sensor snapshots
under their owners' mutex/critical section as needed. Queue operations copy the
complete packed packet with RTOS synchronization, so no caller-owned pointers
are retained. Never use this task API from an ISR. Queue handles, lengths and
RTOS descriptors retain the types required by the kernel ABI.

Each queue (telemetry, command, mapping, barcode) holds four maximum-sized
packets. Telemetry enqueue is nonblocking;
E_OK means queued, not delivered. E_TMOUT indicates full, E_NOSPT networking
not compiled in, E_PAR invalid packet. Incoming commands are dropped if their
queue is full. One pending telemetry packet is retained across immediate lwIP
publish failures. Asynchronous failures are logged, without durable replay.

## Broker, topics and build

Keep credentials in the ignored `config/wifi_secrets.h`, copied from the example.
Your Wi-Fi/broker settings remain valid. Build with:

```sh
make -C build_make PICO_SDK_PATH=/path/to/pico-sdk NET=lwip MQTT=1 TEST=1
```

| Topic | Payload | QoS / retained |
| --- | --- | --- |
| `MQTT_TOPIC` from secrets | One byte online every 2 seconds; offline Last Will | 1 / yes |
| `<MQTT_CLIENT_ID>/telemetry` | Exactly 15 binary bytes | 1 / no |
| `<MQTT_CLIENT_ID>/command` | Exactly 1 byte, code 0–5 | subscription QoS 1 |
| `<MQTT_CLIENT_ID>/barcode` | Exactly 4 binary bytes, one per completed/rejected scan | 1 / no |

Override command/telemetry/barcode topics in secrets if needed. Reconnect installs the
subscription again. RX validates exact topic, declared length, fragment count
and reserved bits; binary zeros are preserved. Offline Last Will explicitly
sets length 1 so its zero byte is not mistaken for an empty string.
Old JSON/text publishers are incompatible. Clear old retained text heartbeats
on your broker if you have any. Do not retain commands.

## Commands across a connection loss (#49)

The intended behaviour, recorded 22 Nov 2026 (`DECISIONS.md`) and tested on
the robot by S1-U20 (`testing/subsystem1/TEST_LOG.md`):

1. **A command or map sent while the robot is offline is dropped, never
   replayed.** The robot connects with a clean session every time (lwIP's
   MQTT client always sets the flag), so the broker keeps nothing for it and
   delivers nothing old after a reconnect. To act on it, send it again once
   the robot is back.
2. **Check the robot is online before sending.** `MQTT_TOPIC` carries `01`
   every 2 s while connected and the retained Last Will `00` once the broker
   drops the robot. The broker can take up to 1.5 × the 60 s keepalive (90 s)
   to notice; anything sent in that window is lost the same way.
3. **Every command is safe to repeat.** QoS 1 may deliver a command twice,
   and the operator may press a button twice:

   | Command | First time | Repeated |
   | --- | --- | --- |
   | STATUS (0) | sends a telemetry packet | sends another |
   | PAUSE (1) | stops the motors (in Ready: cancels a pending START) | nothing more changes |
   | RESUME (2) | resumes, if the IMU is healthy | `RESUME ignored: not paused` |
   | START (5) | arms the robot | `START ignored: already started` |
   | RESET_STATS (3), REVERSE (4) | not supported in `integrated`: logged, no effect | same |
   | letter map | the newest valid map is applied | the same map changes nothing |

4. **Order is kept, age is not checked.** Commands carry no sequence number
   or timestamp, so the robot cannot tell an old command from a new one.
   Senders must therefore not queue commands to send later: the dashboard and
   `tools/` publish each command with its own `mosquitto_pub` call, which
   fails instead of queueing when the broker is down. Never retain a command
   or a map: a retained one would be applied again on every reconnect.
5. **Commands already received are still applied.** Up to four can wait in
   the command queue when the link drops; the control loop empties it every
   cycle (milliseconds). A full queue drops and counts the extra commands
   (`[S1:CMD] commands rx=..`, S1-U17).
6. **A reboot forgets the map.** It is not retained or stored, so after any
   reboot send the map again before START; until then the robot goes
   straight at junctions (#72).

Not tested: a link blip short enough that the robot's TCP connection
survives. TCP would then deliver a command a few seconds late, in order, and
the robot would apply it on arrival. S1-U20 covers a 30-40 s outage, where
the connection does not survive.

## Mac testing without the Pico

`tools/mqtt_binary.py` needs only Python's standard library. It converts binary
to JSON for display; firmware and MQTT payloads remain binary.

```sh
# Decode telemetry as it arrives (Mosquitto hex output, one packet per line).
mosquitto_sub -h localhost -t 'group17-pico/telemetry' -F '%x' | python3 tools/mqtt_binary.py stream
# Send a raw status command. -s reads binary stdin, including byte 0.
python3 tools/mqtt_binary.py command status | mosquitto_pub -h localhost -t 'group17-pico/command' -s
# Simulate telemetry for testing the display; missing fields default to zero.
python3 tools/mqtt_binary.py encode '{"speed_cms":50,"highest_hump_mm":12}' | mosquitto_pub -h localhost -t 'group17-pico/telemetry' -s
# Inspect heartbeat bytes: expect 01 online, 00 offline.
mosquitto_sub -h localhost -t 'group17-pico/heartbeat' -F '%x'
```

Ranges are named constants in `communication_protocol.h`. If a changed range
needs a different bit width, also change layout/offsets/packet size in the C
codec, Python decoder, tests and this table. Compile-time checks catch range
allocations that no longer use the minimum bits. Protocol v1 has no version byte;
coordinate incompatible revisions with all publishers and subscribers.

See `testing/subsystem1/BINARY_CHANGE.md` for measured before/after results and
remaining hardware evidence. Radio recovery, live sensor integration and final
RTOS timing must still be tested on the Pico.
