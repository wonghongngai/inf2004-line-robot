# Minimum-bit change evidence — 2026-10-27

The field representation table is in `app_program/subsystem1/README.md`.

## Comparable firmware builds

Command used before and after:

```
make -C build_make PICO_SDK_PATH=/Users/aliciakan/pico-sdk NET=lwip MQTT=1 TEST=1 -j4
arm-none-eabi-size build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_test.elf
```

| ELF section | Before (bytes) | After (bytes) |
| --- | ---: | ---: |
| text (code + read-only data) | 338436 | 338960 |
| data | 96 | 96 |
| bss | 82472 | 82092 |
| Static RAM reservation (data + bss) | 82568 | 82188 |

Static RAM reservation fell 380 bytes; text increased 524 bytes, including changed
on-target tests. These are build measurements, not runtime high-water marks.

| Queue backing storage requested | Before | After |
| --- | ---: | ---: |
| Telemetry queue, depth 4 | 784 | 64 |
| Command queue, depth 4 | 784 | 32 |
| Total | 1568 | 96 |

The kernel stores a 4-byte length and aligns each payload to 4 bytes. Queue
requests shrink 1472 bytes. These are calculated from the actual allocation
expressions and kernel alignment, not a runtime heap measurement. They free
space INSIDE the existing kernel heap; do not add this number to the ELF RAM
saving. Control blocks, allocator overhead, stack high-water marks and other
subsystems are not included. Capture runtime free heap/stack evidence on target.

## Message size comparison

| Message | Before | After |
| --- | --- | --- |
| Telemetry | Variable JSON, at most 191 bytes | Exactly 10 bytes for all 11 fields |
| Command | Variable text, e.g. `status` = 6 bytes | 1 byte (2 information bits) |
| Online heartbeat | `{"status":"online","sequence":0}` = 32 bytes | 1 byte (1 information bit) |
| Offline Last Will | `{"status":"offline"}` = 20 bytes | 1 byte |

Telemetry is 94.8% smaller than the old maximum permitted payload, not a
like-for-like measured telemetry sample. The earlier demonstration JSON
`{"state":"moving","speed_mps":0.5}` was 34 bytes and carried fewer fields.
MQTT topic names, fixed headers, TCP/IP and radio overhead are excluded.
The old unbounded heartbeat sequence was removed because status is all that is
needed for heartbeat; it was not part of the agreed telemetry fields.

## Correctness evidence

- Host C test exhausts every legal value in every field, verifies neighboring
  fields are unchanged, rejects out-of-range writes and invalid wire encodings,
  and checks all 256 command bytes. Passed with address/undefined sanitizers.
- C encoder and independent Python encoder/decoder agree on the mixed-boundary
  fixture `ffe426ffffe73f645ac8`.
- Full Wi-Fi/MQTT RTOS firmware builds successfully with on-target tests enabled.
- On-target tests cover packet sizes, range rejection, queue handling of a zero
  command byte, malformed lengths and reserved command bits. Built, NOT run.

Reproduce the host codec test:

```sh
cc -std=c11 -O2 -Wall -Wextra -Werror -fsanitize=address,undefined -Iapp_program/subsystem1 testing/subsystem1/protocol_test.c app_program/subsystem1/communication_protocol.c -o /tmp/s1-protocol-test
/tmp/s1-protocol-test
python3 -m unittest discover -s testing/subsystem1 -p 'test_*.py'
```

## Performance and hardware evidence still required

Packing and validation use bounded loops over at most 80 field bits; there is no
heap allocation in the codec and queue calls remain nonblocking. This is a design
property, not measured RTOS timing. Measure packing/publishing latency, radio
reconnection, command delivery including fragmentation/invalid packets, queue
saturation, live highest-hump reporting and control-task timing on the Pico.
Record actual timing and RAM high-water marks before claiming Week 13 completion.
