# Subsystem 1 — test log

**Subsystem:** Wi-Fi, MQTT, telemetry  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

**Rewritten 20 Nov (#14).** Until then this table still described the course
template's LED tests under S1-U01 to U03. Those template tests were never run,
so no results are lost; the IDs now describe what `subsystem1_test.c` checks.
The reverse-command check added to `subsystem1_test.c` on 18 Nov (`9618a21`)
printed `S1-U11`, which `TEST_PLAN.md` had already given to the Wi-Fi join test
on 13 Nov (#15, DEVIATIONS B1-1). It was renumbered **S1-U18** on 20 Nov.
Results recorded before then that list S1-U11 refer to S1-U18.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | Binary packet sizes: `S1_TELEMETRY` is 15 bytes (118 bits, `subsystem1/README.md`) and `S1_COMMAND` 1 byte, with no padding | CR1, IF-05, IF-06 | auto | `make TEST=1`, `subsystem1_test.c` (expected 10 bytes until 20 Nov; the packet grew to 15 bytes in `9618a21`) | `[TEST] S1-U01 PASS` |
| S1-U02 | A freshly initialised telemetry packet reads back logical speed 0 | CR1, IF-05 | auto | `make TEST=1`: `s1_telemetry_init`, then `s1_telemetry_get(SPEED_CMS)` | `[TEST] S1-U02 PASS` |
| S1-U03 | An out-of-range speed (201 cm/s) is rejected by the setter | CR1, IF-05 | auto | `make TEST=1`: `s1_telemetry_set(SPEED_CMS, 201)` returns 0 | `[TEST] S1-U03 PASS` |
| S1-U04 | Publishing a NULL telemetry packet is rejected | IF-05 | auto | `make TEST=1`: `subsystem1_publish_telemetry(NULL)` | returns `E_PAR`; `[TEST] S1-U04 PASS` |
| S1-U05 | Polling an empty command queue does not block | IF-06 | auto | `make TEST=1`: `subsystem1_receive_command(&cmd, TMO_POL)` with nothing queued | returns `E_TMOUT` at once; `[TEST] S1-U05 PASS` |
| S1-U06 | A STATUS command (a binary zero byte) survives the queue intact | FR1-3, IF-06 | auto | `make TEST=1`: `s1_deliver_command({STATUS}, 1)`, then receive | 1 byte received, equal to `S1_COMMAND_STATUS` |
| S1-U07 | A command with reserved bits set (value 5) and a 2-byte command are both rejected before queueing | FR1-3, IF-06 | auto | `make TEST=1`: deliver both, then receive with `TMO_POL` | queue empty: `E_TMOUT` |
| S1-U08 | 3-byte barcode map round trip: A=left, B=right, C=straight, Z=U-turn | CR1, IF-12 | auto | `make TEST=1`: `s1_mapping_set` then `s1_mapping_get` | `sizeof(S1_MAPPING)` = 3; bytes `20 88 0c`; letters 0, 1, 2, 25 read back |
| S1-U09 | A map that uses the same letter twice is rejected and leaves the packet unchanged | IF-12 | auto | `make TEST=1`: `s1_mapping_set` with letters A, B, A, Z | returns 0; byte 2 still `0c` |
| S1-U10 | A valid map is queued; a map with reserved bit 20 set is rejected | IF-12 | auto | `make TEST=1`: deliver the valid map, then the same map with bit 20 set; receive twice | first receive returns 3 bytes ending `0c`; second returns `E_TMOUT` |
| S1-U11 | Wi-Fi joins after boot | FR1-1 | manual (planned) | `TEST_PLAN.md`, Buddy 1 (#15) | `TEST_PLAN.md` |
| S1-U12 | Telemetry publish rate and validity | FR1-2, IF-05 | manual (planned) | `TEST_PLAN.md`, Buddy 1 (#15) | `TEST_PLAN.md` |
| S1-U13 | Reconnect after a broker restart and after the AP goes off and on | FR1-4, NFR1-2 | manual (planned) | `TEST_PLAN.md`, Buddy 1 (#15) | `TEST_PLAN.md` |
| S1-U14 | PAUSE / RESUME / STATUS from the laptop | FR1-3, IF-06 | manual (planned) | `TEST_PLAN.md`, Buddy 1 | `TEST_PLAN.md` |
| S1-U15 | Letter-to-movement map sent over MQTT: valid, duplicate letters, reserved bits set | IF-12 | manual (planned) | `TEST_PLAN.md`, Buddy 1 (#38) | `TEST_PLAN.md` |
| S1-U16 | Barcode result events on `<id>/barcode` | IF-13 | manual (planned) | `TEST_PLAN.md`, Buddy 1 | `TEST_PLAN.md` |
| S1-U17 | A command flood does not starve control | NFR1-3 | auto/manual (planned) | `TEST_PLAN.md`, Buddy 1 | `TEST_PLAN.md` |
| S1-U18 | A REVERSE command survives the queue intact (printed as S1-U11 before 20 Nov) | FR1-3, IF-06 | auto | `make TEST=1`: `s1_deliver_command({REVERSE}, 1)`, then receive | 1 byte received, equal to `S1_COMMAND_REVERSE` |

Host (PC) tests, run with `sh testing/run_host_tests.sh s1/`:

| Name | What it checks |
|---|---|
| `s1/protocol` | telemetry codec in `communication_protocol.c`: field packing, ranges, failed setters leave the packet unchanged |
| `s1/binary_cli` | `tools/mqtt_binary.py` encoder/decoder |
| `s1/barcode_queue` | barcode-over-MQTT queue. Known failure: the code it tests is not committed (#3) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).
*(planned)* = defined but not yet runnable.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-19 | S1-U02–U10, U18 (printed as S1-U11) **on target** | `d9c4830` | robot operator; log analysis Claude | PASS (10 checks) | [`../subsystem3/logs/2026-11-19_target_test.txt`](../subsystem3/logs/2026-11-19_target_test.txt) | `robot-integrated-test.uf2` (`sh build.sh integrated TEST=1`), boot 22:29; the full test run for all subsystems. First on-target run of the S1 checks. `subsystem1 SUMMARY 10 passed, 1 failed` |
| 2026-11-19 | S1-U01 **on target** | `d9c4830` | robot operator; log analysis Claude | FAIL | same log, line `[TEST] S1-U01 FAIL  binary packet sizes` | The check still expected 10-byte telemetry. The packet has been 15 bytes since `9618a21` (18 Nov), as `subsystem1/README.md`, `tools/mqtt_binary.py` and the `_Static_assert` in `communication_protocol.c` all say, so the test was stale, not the code. Expectation changed to 15 on 20 Nov (#14); **re-run on the Pico** |
| 2026-11-20 | `s1/protocol`, `s1/binary_cli` | `65ba452` | Claude, host | PASS | [`logs/2026-11-20_host.txt`](logs/2026-11-20_host.txt) | `sh testing/run_host_tests.sh -v s1/` on a clean tree. Host only; no sanitizers |
| 2026-11-20 | `s1/barcode_queue` | `65ba452` | Claude, host | FAIL (known, #3) | [`logs/2026-11-20_host.txt`](logs/2026-11-20_host.txt) | Compile error: `S1_BARCODE_RESULT`, `barcode_queue` and `subsystem1_publish_barcode` are not in the committed code. Listed in `known_failure()` |
| 2026-11-20 | S1-U01–U10, U18 **on target**, Wi-Fi on | `65ba452` + uncommitted `chore/m0-housekeeping` changes | Claude (flash + capture); robot on a stand, set up by the user | PASS (11 checks, 0 failed) | [`../subsystem4/logs/2026-11-20_target_test.txt`](../subsystem4/logs/2026-11-20_target_test.txt) | `sh build.sh integrated TEST=1`. Re-run after the S1-U01 fix: `[TEST] S1-U01 PASS  binary packet sizes (15-byte telemetry, 1-byte command)`; the reverse check now prints as S1-U18. Wi-Fi could not join the hotspot during the run (not needed by these checks) |
| 2026-11-20 | S1-U01–U10, U18 **on target**, Wi-Fi off | `65ba452` + uncommitted `chore/m0-housekeeping` changes | Claude (flash + capture); robot on a stand, set up by the user | PASS (11 checks, 0 failed) | [`../subsystem4/logs/2026-11-20_target_test_offline.txt`](../subsystem4/logs/2026-11-20_target_test_offline.txt) | `sh build.sh integrated TEST=1 WIFI=none NET=none MQTT=0` |
| 2026-11-20 | S1-U01–U10, U18 **on target**, 20 ms IMU build | `65ba452` + uncommitted `chore/m0-housekeeping` changes (20 ms period) | Claude (flash + capture); robot on a stand, set up by the user | PASS (11 checks, 0 failed) | [`../subsystem4/logs/2026-11-20_target_test_20ms.txt`](../subsystem4/logs/2026-11-20_target_test_20ms.txt) | Same run as the S4 20 ms re-test; Wi-Fi joined |
