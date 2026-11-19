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
| S3-U36 | Three 19 Nov hardware scans of A with scan speed changing within characters decode A, forward and reversed (rank-based narrow/wide rule) | FR3 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U36 PASS` |
| S3-U37 | A frame whose wide/narrow bar separation is below 1.15x is rejected | FR3 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U37 PASS` |
| S3-U38 | Three 19 Nov hardware scans of the medium-size A (~0.73x print) decode A, forward and reversed | FR3 | auto | `line_barcode_test.c` host + target | `[TEST] S3-U38 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-19 | S3-U02–U13, U14–U22, U25–U26, U36–U38 | `875fa82` | Claude, host | PASS (26 checks) | [`logs/2026-11-19_host.txt`](logs/2026-11-19_host.txt) | `sh testing/run_host_tests.sh -v s3/`: s3/line_barcode, s3/pptx_samples, s3/count_a all PASS. Host only; no sanitizers. U14–U17 here are the `line_barcode_test.c` checks of those IDs, not the `subsystem3_test.c` target-only ones |
| 2026-11-19 | all_test_offline and all_test firmware builds | `875fa82` | Claude, compiler | PASS, 0 warnings | [`logs/2026-11-19_build.txt`](logs/2026-11-19_build.txt) | Builds only; no hardware execution |

### Hardware barcode trials, 19 Nov (S3-M03)

Robot running `all_test` offline (default map A=LEFT, F=RIGHT, Z=STRAIGHT, W=U-TURN), scanner GP28 in SIDE layout. All counts and widths come from [`logs/2026-11-19_barcode_hw.txt`](logs/2026-11-19_barcode_hw.txt), a trimmed copy of `testing/subsystem5/logs/2026-11-19_183720_console.txt` (11 MB, gitignored). "Decoded" counts frames of 5 runs or more. "Bias" is how much shorter bars read than spaces of the same printed width (ideal 0). These rows are partial evidence for S3-M03: one letter each and one speed, with no agreed recognition-rate target, so S3-M03 is not complete. The GP28 comparator was re-adjusted between rows as noted.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-19 | S3-M03 large A, forward | image built 18:35, before `a3917a2` | robot operator; log analysis Claude | FAIL (0/5) | `barcode_hw` 18:38–18:42 | Max 25/29 runs. GP28 biased toward black: narrow white spaces lost, merging two bars into one. Hardware fix: comparator rebalanced |
| 2026-11-19 | S3-M03 large A, forward | same image | as above | FAIL (1/4) | `barcode_hw` 19:18–19:22 | 3 frames reached 29/29; 2 rejected by the old strongest-gap ratio split (scan speed changed within a character). Fixed in `a3917a2` (rank-based rule); replaying those widths through it decodes all 3 |
| 2026-11-19 | S3-M03 large A, forward | `a3917a2` | as above | PASS (3/3) | `barcode_hw` 19:34–19:37 | Weakest wide/narrow separation 1.46x; the old rule would have rejected 2 of these 3 |
| 2026-11-19 | S3-M03 large A, reversed (sheet flipped) | `a3917a2` | as above | FAIL (0/5) | `barcode_hw` 19:47–19:49 | Narrow bars read 6–33 ms or vanished; merged whites passed the 1000 ms run limit and aborted capture. Fixed in `436a960` (scan speed 0.75x → 0.9x) |
| 2026-11-19 | S3-M03 large A, reversed | `a3917a2` + scan scale 0.9 (later part of `436a960`) | as above | FAIL (3/4) | `barcode_hw` 19:55–19:57 | All 29/29. One rejected: a 165 ms narrow space read 391 ms during six LEFT corrections (left-wheel pivot slowed the scanner side). Fixed in `436a960` (no pivot while scanning) |
| 2026-11-19 | S3-M03 large A, reversed | `436a960` | as above | PASS (3/3) | `barcode_hw` 20:02–20:04 | Narrow-space spread within a frame ≤ 1.18x (was up to 2.68x); separation ≥ 2.04x; line not lost |
| 2026-11-19 | S3-M03 medium A (~0.73x print), reversed | `436a960` | as above | FAIL (0/4) | `barcode_hw` 20:11–20:13 | Bias ~65 ms: every narrow bar lost (11 runs = the 6 wide bars). Hardware fix: comparator toward black |
| 2026-11-19 | S3-M03 large A + medium A, reversed | `436a960` | as above | large PASS (3/3); medium FAIL (0/3) | `barcode_hw` 20:15–20:19 | After first retune: bias ~55 ms. Medium narrow bars 6–36 ms: some below the 10 ms minimum, some lost (27/29 runs) |
| 2026-11-19 | S3-M03 large A + medium A, reversed | `436a960` | as above | PASS (6/6: 3 large, 3 medium) | `barcode_hw` 20:23–20:25 | Bias ~37 ms. Medium narrow bars ≥ 28 ms. Medium widths saved as S3-U38 |
| 2026-11-19 | S3-M03 small Y (~0.57x print), forward | `436a960` | as above | FAIL (0/4) | `barcode_hw` 20:33–20:37 | Only the 6 wide bars seen (11 runs); narrow bars (~1.7 mm) thinner than the sensor spot. Not resolved; small size dropped from scope |
| 2026-11-19 | S3-M03 small Y | `436a960` | as above | FAIL (no frames) | `barcode_hw` 20:39–20:42 | Sensor lowered and over-tuned: GP28 read black continuously, 0 edges. Height restored for hump clearance |
| 2026-11-19 | S3-M03 large A + medium A, reversed | `436a960` | as above | large PASS (3/3); medium FAIL (0/2) | `barcode_hw` 20:43–20:51 | Bias ~62 ms after the over-tune was backed off. A 9-run frame at 20:43:54, during tuning, is not counted |
| 2026-11-19 | S3-M03 medium A, reversed | `436a960` | as above | FAIL (0/19) | `barcode_hw` 20:57–21:20 | Comparator retuning in steps; bias ranged ~55–95 ms. Narrow bars lost or below 10 ms |
| 2026-11-19 | S3-M03 medium A, reversed | `436a960` | as above | PASS (4/4) | `barcode_hw` 21:20–21:22 | Bias ~48 ms; narrow bars ≥ 20 ms; separation ≥ 2.05x |
| 2026-11-19 | S3-M03 medium A, reversed | `436a960` | as above | FAIL (0/2) | `barcode_hw` 21:25:00, 21:25:31 | Bias ~50 ms; all 29/29 but one narrow bar of a `*` read 7–8 ms (< 10 ms minimum) |
| 2026-11-19 | S3-M03 medium A, reversed | `436a960` | as above | PASS (2/2) | `barcode_hw` 21:26–21:27 | Final comparator setting: bias ~10 ms; narrow bars 49–70 ms, narrow spaces 82–86 ms |
| 2026-11-19 | S3-M03 large A, reversed | `436a960` | as above | PASS (1/1) | `barcode_hw` 21:29 | Final setting: bias ~13 ms; narrow bars ≥ 77 ms; separation ≥ 2.63x |

Other events in the same session, not counted above: a start-up line-sensor timing fault (`sample_dt=23ms line_fault=4`) before any barcode, 4 times in about 40 boots; "IMU collision detected" stops after some runs; one boot (20:54) with Buddy 4 "accelerometer not responding" errors. In every successful run above, the robot then turned LEFT at the next junction as mapped. That is not INT-08 evidence, which needs the integrated firmware and 4 commands × 5.

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
