# Subsystem 3 — test log

**Subsystem:** Line following and barcode; retained consumer adapter  ·  **Student ID:** 2502871  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | statistics self-consistent (min ≤ max, 0..100) | | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |

| S3-U02 | All eight input patterns map to correct position | FR3 / CR1 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U02 PASS` |
| S3-U03 | One sample of noise is rejected | FR3 / CR1 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U03 PASS` |
| S3-U04 | Persistent line loss enters search | FR3 / CR1 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U04 PASS` |
| S3-U05 | Reacquisition returns to tracking | FR3 / CR1 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U05 PASS` |
| S3-U06 | Search timeout reports a line fault | FR3 / CR1 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U06 PASS` |
| S3-U07 | A-Z frames decode forward and backward (expanded after sample review) | FR3 / CR1 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U07 PASS` |
| S3-U08 | Sampled frames report exactly one decoded letter | FR3 / CR1 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U08 PASS` |
| S3-U09 | Malformed zero-width frame rejected | FR3 / CR1 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U09 PASS` |
| S3-U10 | Invalid inter-character gap rejected | FR3 / CR1 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U10 PASS` |
| S3-U11 | Missed sampling interval reports timing fault | FR3 / CR1 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U11 PASS` |
| S3-U12 | Wire schema 2 is exactly three bytes with specified bits | FR3 / CR1 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U12 PASS` |
| S3-U13 | 40, 100 and 200 ms narrow widths decode | FR3 / CR1 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U13 PASS` |
| S3-U14 | Equal black/white calibration rejected | FR3 / CR1 | auto | `subsystem3_test.c` target only | `[TEST] S3-U14 PASS` |
| S3-U15 | Null snapshot rejected | FR3 / CR1 | auto | `subsystem3_test.c` target only | `[TEST] S3-U15 PASS` |
| S3-U16 | Controller pause succeeds | FR3 / CR1 | auto | `subsystem3_test.c` target only | `[TEST] S3-U16 PASS` |
| S3-U17 | Paused task maintains fresh heartbeat | FR3 / CR1 | auto | `subsystem3_test.c` target only | `[TEST] S3-U17 PASS` |
| S3-U25 | SIDE cross-junction reported once; slow-scan wide bars and short black patches are not junctions | FR3-2 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U25 PASS` |
| S3-U26 | Decoded letter (both directions) is stored and becomes a COMMAND at the next junction | FR3-3 / instructor demo brief | auto | `line_barcode_test.c` host + target | `[TEST] S3-U26 PASS` |
| S3-U27 | Four-letter run-time mapping accepted; duplicate/lowercase rejected | FR3-3 / MQTT mapping | auto | `subsystem3_test.c` target only | `[TEST] S3-U27 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |

## Physical and integration cases (not yet run)

| ID | Steps | Acceptance / evidence | Status |
|---|---|---|---|
| S3-M01 | Measure GP7/27/0 on black and white, calibrate, sweep a line under each sensor | Correct raw levels and left/centre/right snapshot; saved serial log | Pending hardware |
| S3-M02 | Follow actual bends and junctions under expected lighting/speeds | No unintended track departure; TRACK-profile junction candidates verified; SIDE junction limitation recorded | Pending controller/hardware |
| S3-M03 | Scan the actual barcodes in both directions and at several speeds | Correct decoded letter and configured command, no duplicates; agree recognition-rate target and save trial counts/video | Pending mounting, mapping and physical trials |
| S3-M04 | Remove line, reacquire it, then repeat with no line | SEARCH after 100 ms, FOUND on recovery, fault after 2 s; stop handled by controller | Pending hardware |
| S3-M05 | Pause during avoidance, then resume after bypass | No stale command after pause; correct recovery; controller/encoder enforces 50 cm maximum | Pending integration |
| S3-M06 | Fill event buffer without consuming, and delay IR task over 20 ms | Latched fault in snapshot; controller stops, explicit reset restores service | Pending integration |
| S3-M07 | Measure wake-to-publish timing with all buddy tasks running | 10 ms nominal sampling; no intervals over 20 ms; measure worst case, CPU and RAM before/after | Pending hardware |

Host results below refer to source content recorded by SHA-256 in the log. They
are development evidence on an uncommitted working tree, not an on-target run
or a replacement for the project's committed hardware test records.

| Date | Test IDs | Revision | Runner | Result | Evidence |
|---|---|---|---|---|---|
| 2026-10-25 | S3-U02–S3-U13 | 83664de + working tree; source SHA-256 in log | Codex, host simulator | PASS (12 checks), ASan/UBSan clean | [Host log](logs/2026-10-25_host.txt) |
| 2026-10-25 | S3-U01, S3-U14–S3-U17 and physical cases | working tree | Not run | PENDING hardware | No target result claimed |

Normal and test firmware compilation/link/UF2 generation passed on 2026-10-25.
New core/test/service modules also passed strict ARM warning checks. See
[build evidence](logs/2026-10-25_build.txt). Compilation does not execute the
on-target API tests S3-U14–S3-U17. Baseline/implemented image RAM figures and
remaining runtime measurements are explained in the subsystem README.

## Barcode sample revision (wire schema 2)

The older saved result above used the provisional two-byte A-D interface. Its
log is retained unchanged. The revised three-byte interface carries an A-Z
letter independently of an optional mapped command. S3-U07/U08/U12 now check
the expanded alphabet/report and revised wire contract; their test IDs are kept.

| ID | Check | Execution | Expected |
|---|---|---|---|
| S3-U18 | Decode actual PDF vector-derived A/Z fixtures and reversed Z | host + target | Correct letters |
| S3-U19 | Unmapped letters never command; map all 26 letters across packed byte boundaries; clear and invalid inputs | host + target | Exact command/letter fields; ten-byte map |
| S3-U20 | Toggle scanner while both line inputs remain black | host + target | Centred position, no junction event |
| S3-U21 | Lose line after verified barcode start | host + target | SEARCH still occurs |
| S3-U22 | Select three-sensor TRACK profile, feed junction pattern; reject invalid profile | host + target | Junction state; invalid profile rejected |
| S3-U23 | Configure and clear a letter mapping while paused | target only | E_OK |
| S3-U24 | Reject lowercase mapping and unknown layout | target only | E_PAR |

The SIDE profile requires one offset scanner and two line-edge sensors. Reliable
classification of centred line versus junction is not claimed with only those
two line inputs. This hardware limitation must be resolved before full FR3-2
acceptance. Track orientation and the letter-to-motion mapping remain team inputs.

| Date | Checks | Revision | Runner | Result | Evidence |
|---|---|---|---|---|---|
| 2026-10-25 | S3-U02–U13, U18–U22 | 83664de + working tree; source/fixture SHA-256 in log | Codex, host | PASS (17 checks), ASan/UBSan clean | [Sample-revision host log](logs/2026-10-25_barcode_host.txt) |
| 2026-10-25 | Normal/test ARM builds and strict warning checks | same source revision | Codex, compiler | PASS; no hardware execution | [Sample-revision build log](logs/2026-10-25_barcode_build.txt) |
| 2026-10-25 | S3-U23–U24 | same source revision | Not run | PENDING on-target execution | No hardware result claimed |
| 2026-10-26 | S3-U02–U13, U18–U22, U25–U26 | 83664de + working tree; SHA-256 in log | Claude, host | PASS (19 checks), ASan/UBSan clean | [Junction host log](logs/2026-10-26_junction_host.txt) |
| 2026-10-26 | Normal/test ARM builds and strict warning checks | same source revision | Claude, compiler | PASS; no hardware execution | [Junction build log](logs/2026-10-26_junction_build.txt) |
| 2026-10-26 | S3-U27, bench demo printout | same source revision | Not run | PENDING on-target execution | No hardware result claimed |
| 2026-10-26 | S3-U02–U13, U18–U22, U25–U26 — **BEFORE version** (no minimum-bit change) | 83664de + working tree; SHA-256 in log | Claude, host | PASS (19 checks), ASan/UBSan clean | [Before host log](logs/2026-10-26_before_host.txt) |
| 2026-10-26 | Normal/test ARM builds, strict warnings, symbol sizes — **BEFORE version** | same source revision | Claude, compiler | PASS; no hardware execution | [Before build log](logs/2026-10-26_before_build.txt) |
| 2026-10-27 | S3-U02–U13, U18–U22, U25–U26 — minimum-bit (CR1) version restored; fault field now a 3-bit bitmask | working tree; SHA-256 in log | Claude, host | PASS (19 checks), ASan/UBSan clean | [CR1 host log](logs/2026-10-27_cr1_host.txt) |
| 2026-10-27 | Normal/test ARM builds, strict warnings, symbol sizes — CR1 version | same source revision | Claude, compiler | PASS; no hardware execution | [CR1 build log](logs/2026-10-27_cr1_build.txt) |
