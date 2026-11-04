# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |
| INT-03 | Robot adapter readiness, steering, junction turns, faults and standalone motion | IF-06, IF-07 | host auto | `sh testing/integration/run_robot_host_tests.sh` | 167 assertions pass |
| INT-04 | Six hardware firmware profiles compile and produce UF2 files | IF-06, IF-07, IF-08 | build | `PICO_SDK_PATH=... sh testing/integration/build_robot_profiles.sh` | Six build PASS lines |
| INT-05 | PWM works after boot with PWM initially held in reset | S2 hardware driver | host auto | `sh testing/integration/run_robot_host_tests.sh` | `motor PWM register regression: PASS` |
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
