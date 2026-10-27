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
| `subsystem1_test.c` | On-target queue tests, enabled by `TEST=1` |

## Representation table

Telemetry is exactly 80 bits / 10 bytes, with no struct padding on the wire.
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

States: idle, following, turning, scanning, avoiding, recovering, finished,
fault (0–7 in that order). Fault bits 0–4: motor stall, encoder fault, IMU
fault, no echo, Wi-Fi loss. Sensors occupy bits left=0, centre=1, right=2;
agree whether 1 means black with Buddy 3 (proposed convention: 1 = black).
Fault flags describe the controller's snapshot, not automatic driver updates.

Command codes: 0 request status, 1 pause, 2 resume, 3 reset statistics.
Each occupies bits 0–1 of one byte; upper six bits MUST be zero. A heartbeat
uses bit 0 of one byte (0 offline / 1 online), with seven reserved zero bits.
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

Both queues hold four maximum-sized packets. Telemetry enqueue is nonblocking;
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
| `MQTT_TOPIC` from secrets | One byte online every 5 seconds; offline Last Will | 1 / yes |
| `<MQTT_CLIENT_ID>/telemetry` | Exactly 10 binary bytes | 1 / no |
| `<MQTT_CLIENT_ID>/command` | Exactly 1 byte, code 0–3 | subscription QoS 0 |

Override command/telemetry topics in secrets if needed. Reconnect installs the
subscription again. RX validates exact topic, declared length, fragment count
and reserved bits; binary zeros are preserved. Offline Last Will explicitly
sets length 1 so its zero byte is not mistaken for an empty string.
Old JSON/text publishers are incompatible. Clear old retained text heartbeats
on your broker if you have any. Do not retain commands.

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
