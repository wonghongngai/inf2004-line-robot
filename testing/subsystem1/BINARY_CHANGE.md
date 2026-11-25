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
sh testing/run_host_tests.sh s1/
```

This runs `s1/protocol` (C codec, with ASan/UBSan where the compiler supports
them) and `s1/binary_cli` (`test_binary.py`). Build flags live in
`testing/run_host_tests.sh`.

## Performance questions from the October comparison

In the October 10-byte format, packing and validation used bounded loops over
at most 80 field bits. The codec uses no heap allocation and the queue calls
are nonblocking. See the [S1 results](TEST_LOG.md#results) for later checks of
packet validity, reconnect and control timing. The October build-size figures
do not cover current packing latency or runtime RAM high-water marks.
