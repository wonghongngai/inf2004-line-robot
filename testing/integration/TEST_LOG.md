# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | **Retired 20 Nov (#74).** samples flow from S2 to S3 (≥ 5 in 2 s) | template IF-01, IF-02 (removed) | — | removed from `subsystem3_test.c` | Not run any more: it tested the course template's producer/consumer tasks, which are no longer built. Its PASS lines in on-target logs up to 20 Nov tested only those tasks |
| INT-02 | **Retired 20 Nov (#74).** summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | template IF-03 (removed) | — | no longer in `subsystem4_test.c` | Not run: the template's S3 → S4 summary path no longer exists. Never run on this project |
| INT-03 | Robot adapter readiness, steering, junction turns, faults and standalone motion | IF-06, IF-07 | host auto | `sh testing/run_host_tests.sh int/robot_control` (was `run_robot_host_tests.sh` until 2026-11-13) | 167 assertions pass |
| INT-04 | Six hardware firmware profiles compile and produce UF2 files | IF-06, IF-07, IF-08 | build | `PICO_SDK_PATH=... sh testing/integration/build_robot_profiles.sh` | Six build PASS lines |
| INT-05 | PWM works after boot with PWM initially held in reset | S2 hardware driver | host auto | `sh testing/run_host_tests.sh s2/motor_pwm` (was `run_robot_host_tests.sh` until 2026-11-13) | `motor PWM register regression: PASS` |
| INT-06 | Seven firmware profiles including combined motor/IR/IMU diagnostic | IF-06, IF-07, IF-08 | build | `PICO_SDK_PATH=... sh testing/integration/build_robot_profiles.sh` | Seven build PASS lines |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-28 | INT-03 | working tree on `110d293` | Codex | PASS | `logs/2026-10-28_robot_host.txt` | Production adapter/controller with simulated S3/S4; no physical robot test |
| 2026-10-28 | INT-04 | working tree on `110d293` | Codex | PASS | `logs/2026-10-28_robot_builds.txt` | Six UF2 profiles built; standalone motion ELF has no S3/S4 symbols |
| 2026-10-28 | INT-05 | working tree on `110d293` | Codex | PASS | `logs/2026-10-28_pwm_fix.txt` | Real PWM BSP/motor driver against modeled RP2040 registers; includes divider=12.5 |
| 2026-10-28 | INT-06 | working tree on `110d293` | Codex | PASS | `logs/2026-10-28_pwm_fix.txt` | All seven final profiles rebuilt with reset/frequency fixes; physical retest pending |
| 2026-11-13 | INT-03 | `af2da1c` | Claude, host | PASS | `logs/2026-11-13_host_tests.txt` | `int/robot_control`: 175 checks (adds S2_LINE_UNSAFE_MS debounce checks). S5 executor stubbed as idle, so avoidance hand-over is not covered. No physical robot test |
| 2026-11-13 | INT-05 | `af2da1c` | Claude, host | PASS | `logs/2026-11-13_host_tests.txt` | `s2/motor_pwm`, same modelled-register test as 2026-10-28 |
| 2026-11-13 | INT-03 | `4c15097` | Claude, host | PASS | `logs/2026-11-13_host_tests_retire.txt` | `int/robot_control`: 209 checks; adds line-stop re-arm, restart-on-loss, restart-on-wheel-movement and IMU-stop latch (ported from the retired all_buddies tests, #9). S5 executor stubbed as idle |
| 2026-11-20 | INT-03 | `65ba452` | Claude, host | PASS | [`../subsystem2/logs/2026-11-20_host.txt`](../subsystem2/logs/2026-11-20_host.txt) | `int/robot_control`: 209 checks, on a clean tree; run for #17. S5 executor still stubbed as idle |
| 2026-11-20 | INT-03 | `42be6ea` + uncommitted M1 decision changes | Claude, host | PASS | [`logs/2026-11-20_host_tests_decisions.txt`](logs/2026-11-20_host_tests_decisions.txt) | `int/robot_control` after #67 (tilt is report-only): a TILT state and event now leave the robot following; the IMU-stop checks use `IMU_EVENT_FAULT` instead. Full host suite 10 passed, 2 known (#3). S5 executor still stubbed as idle. Not yet run on the robot (INT-11, #36) |
