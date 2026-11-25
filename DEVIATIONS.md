# Deviations from the original plan

Baseline 13 November 2026. Current implementation and test notes reviewed on 25 November 2026. A copy of the team doc "Group 17 Robot: Deviations from the
Original Plan", updated with the #4 decision and the M1 decisions in [DECISIONS.md](DECISIONS.md). Keep this file current in the
same PR as any change that adds, closes or justifies a deviation.

## Purpose and sources

This register lists every place where the current code, or a planned task, differs from what the team committed to, so each change can be justified at Tribunal 2. It compares the code on `main` (commit `6e0658e`, plus the test fixes on `chore/fix-host-tests`) and GitHub issues #3 to #69 against these sources:

| Key | Source | Date |
| --- | --- | --- |
| WU | Project Write-up (mission requirements, buddy tasks, deliverables) | Sep 2026 |
| W6 | Group 17 Week 6 Design Review (FR/NFR, pins, RTOS tasks, statecharts, assumptions) | Week 6 |
| PT | CSC17 PreTri 1, Week 8 Tribunal 1 (decisions, risks, gap check) | Week 8 |
| Prof | Instructor updates on barcodes, MQTT mapping and collision detection | 13 Nov 2026 |

The source documents are not in this repo. Each row gives the planned item, what exists now, the deviation type and the justification or follow-up issue. Deviation types: **Mandated** (instructor changed the requirement), **Changed** (the team chose a different design), **Missing** (planned but not built), **Partial** (built but not verified or incomplete), **Unverified** (built, no recorded evidence).

## Summary

The summary below lists the changes and gaps that most affect the demo.

| # | Deviation | Type | Fix |
| --- | --- | --- | --- |
| 1 | Barcode letter-to-movement map: no longer hard-coded; both builds apply the MQTT map and go straight until it arrives; telemetry bit 86 reports it (21 Nov, host-tested). A–Z on the robot not tested by decision (21 Nov, `DECISIONS.md`): only A was scanned on the robot | Partial | #38 |
| 2 | IMU collision detection removed and the lift-stop built (#53, 21 Nov); host-tested; S4-U12 passed on the robot 21 Nov | Mandated | #53 |
| 3 | The obstacle trigger is 20 cm. The robot must rest at least 10 cm away. | Changed | #64. S5-U48 passed 9/9 on 23 Nov. |
| 4 | Turning, straight-driving and avoidance tuning moved from the test harness to `integrated`. | Changed | #4, #8. See INT-08 and INT-10 for the integrated track trials. |
| 5 | Task priorities changed from Week 6. S5 runs at priority 20. | Changed | #66. S1-U17 and S2-U12 measured control timing with Wi-Fi on. |
| 6 | Reverse by distance built: `MOTION_PROFILE_REVERSE` (FORWARD_CM driven backwards, heading held), used by the S5 back-off (`ea03e1a`, 23 Nov) and the barcode reverse-and-rescan (PR #127, 24 Nov). Host-tested; S2-U13 distances 9 of 9 within ±2 cm; on the robot (25 Nov, `84988ea`, no logs) the barcode rescan worked 3 of 3 (S2-U17), and the S5 back-off worked 3 of 3: two 15 cm back-offs and rescans, then NO PATH (S5-U51) | Partial | #39 |
| 7 | No IMU motion classes or turn rate (FR4-3, FR4-4; still assessed) | Missing | #42, #43 |
| 8 | Coarse scan uses 50/70/90/110/130 deg, FR5-1 specifies 30/60/90/120/150 deg | Changed, decided 20 Nov: FR5-1 updated | #65 (`DECISIONS.md`) |
| 9 | A5-2 tilt suppression only in part: S5 ignores forward readings while S4 reports a hump or tilt (`s5_detect_allowed()`, since 7 Nov), but has no hold after the tilt clears and no rate-of-change check; its test S5-U44 was dropped (#44) | Partial | #44 (decided: no further work) |
| 10 | Test results are recorded at subsystem, integration and full-course levels. Some runs rely on operator observation. | Partial | [Current evidence](STATUS.md) |

## Changes mandated by the instructor

Three requirement changes came from the instructor after Week 6; only the minimum-bit change is fully reflected in the code.

| ID | Original plan | New requirement | Current code | Status | Issue |
| --- | --- | --- | --- | --- | --- |
| M1 | WU p3: barcodes A to D map to fixed commands; W6 FR3-3 decodes "navigation barcodes" | Prof, 13 Nov: decode all A to Z in both directions; store the letter; act at the next cross-junction; the 4-letter map is sent PC to car over MQTT just before the demo and must not be hard-coded | Decoder handles A to Z both ways and "store, act at junction" (host tests S3-U07, S3-U26). Map hard-coded: integrated A,B,C,Z; all_test A,F,Z,W and logs "MQTT map ignored" | Partial | #38 |
| M2 | WU p8, W6 FR4-3: detect collision with the IMU; W6 system statechart: collision triggers Emergency Stop | Prof, 13 Nov: IMU collision detection removed; lifting the robot must stop the motors; no detailed lift or overturn classification | Collision detection removed (21 Nov). Lift check: 200 ms average of the Z deviation from the baseline ≥ 200 mg for 60 ms; it stops the robot in both builds and the stop latches. Host tests pass; S4-U12 on the robot passed 21 Nov (7/7 lifts stopped, 0 false stops in 6 hump runs) | Matches | #53 |
| M3 | W6 data passed as plain structs and floats | PT curveball: minimum-bit data representation (CR1) | Done for S1 telemetry (80 bits), S3 messages (3 bytes), S2 motion protocol (5-byte command, 12-byte status with widths decided from measurements 23 Nov, #52), S5 request word; S4 still the "PRE" version | Partial | #25 (S4-U14), #52. S5 distance: 9-bit cm on the wire, mm inside S5 (#54, decided 20 Nov) |

For M1, the team decided that before a map arrives the robot goes straight at junctions.

## Architecture and RTOS

The task structure still matches Week 6 (one task per buddy), but priorities, the inter-task objects and the startup flow have all changed, and recent work bypasses the Week 6 architecture entirely.

| ID | Week 6 / PT plan | Current code | Type | Justification or follow-up |
| --- | --- | --- | --- | --- |
| A1 | W6 §3 priorities: S5 obstacle highest, S4 IMU high/highest, S2 motion high, S3 line high, S1 comms medium | T-Kernel priorities (lower = runs first): Wi-Fi service 4, S2 motion 14, all_test loop 15, S3 line 16, S4 IMU 18, S5 obstacle 20, so S5 is now the **lowest** | Changed | Decided 20 Nov: keep. S2 and S3 run on core 1, Wi-Fi on core 0; S4 timing unchanged with Wi-Fi off; S5 only requests moves (`DECISIONS.md` #66). S1-U17 measured 22 Nov: no late control steps under a command flood (#15). Still to record: S2-U12 (#19) |
| A2 | W6 diagram showed a Vehicle Controller and Safety Monitor as tasks; PT: keep the VC as a function in an existing task | VC runs inside `motion_task` (priority 14, core 1) in the integrated build | Changed (agreed at PT) | Closed; Safety Monitor never built as a separate entity |
| A3 | W6 §4 objects: `line_nav_q`, `imu_data_q`, `obstacle_q`, `motion_cmd_q`, `motion_status_q`, `telemetry_q`, `cmd_q`, one `robot_events` event group | S3 message buffer + snapshot API; S4 snapshot under a mutex (`subsystem4_get_latest`), no queue; S5 packed 32-bit request word + `g_flg_s5_evt` (IF-10/IF-11), no `obstacle_q`; `g_mbf_motion_cmd` + STOP flag; motion status read through a getter; no shared event group | Changed | Interfaces are listed as IF-05 to IF-11 in `TEAM.md`, but sections 1 and 2 are still template rows (#10) |
| A4 | W6: one integrated system coordinated by the VC | `integrated` is the final system. The tuning port landed on 21 Nov. `all_test` remains a separate test harness with its own control loop. | Changed | #4, #8 |
| A5 | W6 §5 system statechart: Initialising, Idle/Ready waits for a start command, Navigating, Emergency Stop resumes when safe or after a user reset | Waits in Ready after initialisation (5 s, IMU calibrated, S5 hold released) until an MQTT START (code 5); `S2_AUTO_START=1` restores auto-start for a demo without Wi-Fi; line stops re-arm automatically; IMU faults need a reboot | Matches (host-tested; bench test pending) | #48 |
| A6 | PT gap check (Fix): each event bit has one producer, a clear/latch rule and a priority | Event-bit table drafted in `TEAM.md` §5 (20 Nov); no shared event group exists | Partial | #50 |
| A7 | PT gap check (Fix): invalid packed commands are rejected and the robot stays stopped, with an integration test | In `integrated`, MQTT commands reach the VC only as S1's 1-byte packets. S1 rejects wrong lengths, reserved codes and reserved bits before its command buffer. The VC drives `motion_control.c` directly; the S2 packed command (`motion_protocol.h`) has no producer there. INT-15 PASS 23 Nov on the PC (`int/bad_commands`: real S1 MQTT callbacks → VC → motion controller) and on the robot (`integrated`: 26 bad payloads in Ready, following and paused, no effect, all counted invalid). The robot keeps its state; stopped stays stopped | Matches | #51 (closes on merge) |
| A8 | PT gap check: safety pre-empts avoidance, and avoidance pre-empts line following | The VC checks safety, then S5, then line following. INT-13 exercised the hand-over on the host. INT-10 exercised takeover and return to line following on the robot. | Tested | #20, #35 |

## Buddy 1: Wi-Fi, MQTT, telemetry

Pico and robot tests cover communication, recovery and the supported commands. The Wi-Fi service runs at a higher priority than the control tasks on a different core.

| ID | Requirement (source) | Current code | Type | Follow-up |
| --- | --- | --- | --- | --- |
| B1-1 | FR1-1 Wi-Fi + MQTT connection (W6) | CYW43 + lwIP + MQTT through the kernel Wi-Fi service; PT noted connectivity "not yet tested" | Verified 22 Nov | #15: S1-U11 5/5 joins in ~4.1 s (22 Nov), S1-U12 PASS (21 Nov) |
| B1-2 | FR1-2 publish telemetry including obstacle scans, encoder counts and speed | 19-byte packets at 1 s intervals. Obstacle distance, scan angle and width populate after a valid scan (S5-U40). Each encoder count is sent in its low 16 bits (S1-U12, S1-U21). | Tested | #40, #41, #72 |
| B1-3 | FR1-3 receive commands (W6) | `integrated` handles STATUS, PAUSE, RESUME and START and applies the letter map. S1-U14, S1-U15 and the Ready/START trial exercised these on the robot. RESET_STATS and REVERSE remain unsupported as MQTT movement commands. | Tested for supported commands | #38, #48, #72 |
| B1-4 | FR1-4 / NFR1-2 reconnect without restart (W6) | Recovery logic in `mqtt_manager.c` (MQTT retry every 3 s) and `cyw43_utk.c` (Wi-Fi rejoin) | Verified 22 Nov | #15: S1-U13 PASS on the robot, telemetry back 2.8 s after a broker restart and 3.1 s after the hotspot; needs PR #102 (S4 core pin, merged into `main` 22 Nov), without which the IMU fault stopped the robot during outages |
| B1-5 | FR1-5 periodic status (W6) | 1-byte heartbeat every 2 s (`MQTT_UTK_HEARTBEAT_MS` = 2000; this row said 5 s until 22 Nov) | Unverified | #15 (seen on the broker during the 22 Nov runs, no saved log) |
| B1-6 | NFR1-3 telemetry must not disturb control; W6 gave comms **medium** priority | Wi-Fi service task priority 4, above motion (14) and line (16) | Changed, decided 20 Nov: keep | #66 (`DECISIONS.md`). Measured 22 Nov (S1-U17 PASS): a 100-command flood left `ctrl late` at 0, and the same boot had 0 late control steps in 66 min with Wi-Fi on, through broker and hotspot outages. S2-U12 also recorded zero late steps during integrated track trials |
| B1-7 | PT gap: define handling of commands during a connection loss | Defined 22 Nov (`app_program/subsystem1/README.md`, `DECISIONS.md`): commands and maps sent while the robot is offline are dropped, never replayed (lwIP always connects with a clean session); every command is safe to repeat; senders check the heartbeat and never queue or retain commands | Verified 22 Nov | #49: S1-U20 PASS on the robot |
| B1-8 | Barcode result events on `<id>/barcode` | The reimplemented queue publishes a 4-byte binary packet. S1-U16 checked barcode events on the robot. | Tested | #3, #16 |
| B1-9 | WU deliverables: communication API, telemetry framework, MQTT documentation, telemetry demo | The API and protocol are documented. The [video index](videos/README.md) links the S1 telemetry and command demo. | Evidence recorded | #55 |

## Buddy 2: Motion control

Motor and encoder drivers follow the PT design. The current straight-driving PID and encoder trim were tuned separately. The controller uses internal reverse moves.

| ID | Requirement (source) | Current code | Type | Follow-up |
| --- | --- | --- | --- | --- |
| B2-1 | FR2-1 motors with encoder feedback (W6); PT: count edges in the ISR | `drv_encoder.c` counts in the IRQ. S2-U05 checked the encoders on the robot. | Tested | #18 |
| B2-2 | FR2-2 / NFR2-2 speed and distance from encoders | Current travel and driven-pivot scales are listed in [CALIBRATION.md](testing/subsystem2/CALIBRATION.md). | Tested | #18 (S2-U06, S2-U07) |
| B2-3 | WU p6 PID speed control | Straight-driving PID uses Kp 6.0, Ki 0.5 and Kd 0. S2-U10 recorded one tuned floor run. Guide-line steering uses separate trim. | Bench tested | #18, #56 |
| B2-4 | FR2-3 / NFR2-1 straight-line correction | Both builds apply encoder trim. The integrated straight-track trials are recorded under S2-U08. | Tested | #8, #19, #23 |
| B2-5 | FR2-4 forward, turn, stop; WU p6 required APIs include `moveBackward(distance)` | Commands: FORWARD_CM, TURN_LEFT_DEG, TURN_RIGHT_DEG, STOP, RESET_FAULT, FOLLOW_LINE. W6 §8.1 dropped backwards on AI advice; the team reversed that (13 Nov) and `moveBackward(distance)` is now FORWARD_CM in `MOTION_PROFILE_REVERSE` (no separate command). Host-tested; S2-U13 distances PASS (25 Nov, no log) | Partial | #39 |
| B2-6 | NFR2-1 to NFR2-3 measurable tolerances (W6 gap marked "Fixed") | Thresholds agreed 20 Nov in `TEST_PLAN.md` (S2-U07 to U13); NFR2 wording itself not yet updated | Partial | #6 (`DECISIONS.md`) |
| B2-7 | PT: encoder counter widths decided from measured range (CR1) | Done 23 Nov (#52, S2-U14): widths from the robot logs, status 30 → 12 bytes, "provisional" removed. Other subsystems not yet migrated to the packed status | Partial | #52 (widths); migration open |
| B2-8 | WU deliverables: motion library, PID tuning report, motion accuracy evaluation | Library exists; no report or evaluation | Missing | #56 |

## Buddy 3: Line following and barcode

The decoder is the best-tested part of the project on the PC, but the sensor layout changed from three line sensors to two plus a barcode scanner, and the integrated controller probes for the line before stopping.

| ID | Requirement (source) | Current code | Type | Follow-up |
| --- | --- | --- | --- | --- |
| B3-1 | FR3-1 three IR sensors steer the robot (W6 §2) | Both builds use SIDE. GP28 scans barcodes. GP27 and GP0 steer. The [pin table](README.md#31-sensors-and-pins) shows this layout. | Changed | #7 |
| B3-2 | W6 statechart 3: on line loss, Search Line mode adjusts movement until the line is found | Probe toward the last seen side, then stop and wait if the probe misses. `integrated` steers in the straddle model (#8): C and R sit either side of the tape and both white is the normal reading, so S3's LOST is ignored. Since #23 the VC detects loss itself: a correction of ≥ 150 ms that clears to white with no black within `S2_LINE_LOST_MM` (150 mm, encoders) starts a soft probe toward the side the tape was last under (`S2_LINE_PROBE_MM`, 100 mm); black ends it, none stops the robot; it re-arms once C or R sees the tape and it is held still 1 s, or after PAUSE + RESUME. Host-tested; two false stops on the track (23 Nov, tape still between C and R) led to the probe, which is not yet run on the robot. `LINE_BARCODE_SEARCH_MS` is 0 in both robot builds | Changed, decided 20 Nov: stop and wait; statechart 3 updated (course has no gaps). Straddle VC gets its own loss check (#23, 23 Nov) | #68 (`DECISIONS.md`); S3-U31 in #23 |
| B3-3 | FR3-2 junction detection | The integrated straddle controller uses centre and right IR inputs at a junction. INT-08 recorded 21/21 mapped junction actions. S3-U25 checks the service-level detector. | Tested | #23, INT-08 |
| B3-4 | FR3-3 decode navigation barcodes (W6); prof: all A to Z, both directions | Decoder handles A to Z both ways on the PC (S3-U07); on hardware only A and Z were tried (`208188b`) | Partial | #23 (S3-U32, S3-U35) |
| B3-5 | W6 statechart 3: an invalid barcode is ignored and line tracking continues | Matches, plus (PR #127, 24 Nov) a rejected barcode of at least 9 runs makes the robot reverse 30 cm (45 cm if 20+ runs) and rescan it, up to 2 retries, then carry on as before. On the robot 25 Nov (S2-U17, no log): 2 runs rescanned and read the barcode, 1 gave up on a damaged print as designed, no false back-up at a junction. The #127 case where a rescan reads 29 runs and is still rejected was not seen, but its cause is still unknown | Changed (addition) | #39 |
| B3-6 | Sampling: periodic sensor processing (W6) | Period changed from 10 ms to 1 ms on 3 Nov (`b276d54`) for barcode resolution | Changed | Record CPU cost in the resource report (#61) |
| B3-7 | NFR3-1 robust to lighting, track and speed changes; NFR3-2 reliable recognition | No track or lighting tests recorded; on-target S3 unit tests passed 19 Nov (d9c4830; S3-U01 retired) | Unverified | #21, #22, #23 |
| B3-8 | WU deliverables: line following, decoding and navigation command interface | The S3 README documents runtime MQTT mapping in both builds. The delivery includes code, interface checks, and the linked S3 demo. | Evidence recorded | #57 |

## Buddy 4: IMU, humps, tilt

Calibration, tilt reporting, tuned hump detection and lift-stop are implemented. Motion classification and turn-rate reporting remain open.

| ID | Requirement (source) | Current code | Type | Follow-up |
| --- | --- | --- | --- | --- |
| B4-1 | FR4-1 calibrate before monitoring (W6) | 1 s stationary baseline at start; on-target check S4-U03 passed 19 Nov (`d9c4830`); baseline values not yet logged on the bench | Partial | #24 |
| B4-2 | FR4-2 terrain events and highest hump height | The estimator retains climb height after descent (#5). The tuned detector passed S4-U09 and S4-U10 on the 10 mm ramp. Tilt calibration is tracked separately. | Hump tested | #25, #27 |
| B4-3 | FR4-3 motion classification (W6); WU p8 events: stationary, accelerating, turning, climbing, descending | Only NORMAL, HUMP, TILT, LIFTED, FAULT safety states | Missing | #42 |
| B4-4 | FR4-3 collision detection (W6) | Removed 21 Nov (#53); the lift check replaces it | Mandated removal (M2) | #53 |
| B4-5 | FR4-4 report turn rate (W6, WU) | No turn rate; the LSM303DLHC has no gyroscope; magnetometer is read but unused | Missing | #43 |
| B4-6 | PT decision: a normal hump must not cause an emergency stop; tilt and hump are informational (`TRIBUNAL_STATUS.md`) | Decided 20 Nov: tilt is report-only. The VC no longer halts on tilt (`robot_control.c`); host test INT-03 passes | Matches (not yet tested on the robot) | INT-11 dropped 22 Nov (#36): the ultrasonic sits above the hump |
| B4-7 | NFR4-1 sample every 20 ms; NFR4-3 cycle within 20 ms | S4-U05 passed on the Pico at 20 ms with zero overruns and a 19 ms maximum. The 10 ms comparison failed. The 23 Nov ramp tests used the 20 ms period. | Tested | #24, #27 |
| B4-8 | NFR4-2 report a hump within 100 ms | Not measured; the collision part of NFR4-2 is void (M2) | Unverified | #27 |
| B4-9 | CR1 minimum-bit representation | S4-U06 checks the 4-byte `IMU_PACKED_MSG`. Before/after figures are still due. | Partial | #25 (S4-U14) |
| B4-10 | WU deliverables: IMU module, hump algorithm, terrain analysis report | No report | Missing | #58 |

## Buddy 5: Ultrasonic scanning and avoidance

The full scan, profile, plan and bypass pipeline exists, but the detection distance and scan angles were changed from FR5-1 and NFR5-1, the range tolerance was revised and the 2 s scan limit was retired, A5-1 (corridor) was accepted unmeasured, and A5-2 (tilt suppression) is only partly implemented.

| ID | Requirement (source) | Current code | Type | Follow-up |
| --- | --- | --- | --- | --- |
| B5-1 | FR5-1 coarse scan at 30, 60, 90, 120, 150 deg (W6, WU) | The revised coarse scan is 50, 70, 90, 110 and 130 deg. The sensor holder cannot reach 30 or 150 deg. | Changed | #65 ([DECISIONS.md](DECISIONS.md)) |
| B5-2 | NFR5-1 detect at 30 cm or more, rest at least 10 cm away | The trigger was changed to 200 mm. S5-U48 passed nine times on 23 Nov. The confirmed readings were 183–199 mm and the rest distances were 177–190 mm. | Changed and tested | #64, #31 |
| B5-3 | FR5-2 return to the region and scan in 5 deg steps | Smooth glide with a ping every 5 deg, window capped by `FINE_BUDGET_POINTS` | Matches | None |
| B5-4 | NFR5-3 detection to action within 2 s | Limit dropped for a smooth fine scan in the 5 Nov scan rework (the `FINE_BUDGET_POINTS` cap is gone); S5-U06 retired 22 Nov by the S5 owner (#29). Last result FAIL, 2299 ms (1 Nov). The bench image prints the time for information | Dropped | — |
| B5-5 | NFR5-6 closest point within 2 cm over 5 to 100 cm | Hardware-timed echo since 27 Oct. Decided 22 Nov by the S5 owner (#29): S5-U02 checks 5 / 15 / 30 / 45 / 60 cm only (100 cm dropped) and passes when > 90 % of readings are within ±5 cm. PASS on that rule; ±2 cm is not met (5 cm spikes, ~2 cm short at 45 cm) | Changed (range and tolerance) | — |
| B5-6 | FR5-3 profile location, width, closest point, side clearances | S5-U31–U38 passed on target. Physical width underestimated the boxes in S5-U07. | Partial | [S5 results](testing/subsystem5/TEST_LOG.md#results) |
| B5-6a | FR5-3 width of the obstacle | S5-U07 (22 Nov): closest point and centre correct, but flat boxes under-read (160 mm → 35–100 mm); marked done by the S5 owner because the test box has angled sides | Accepted (owner) | — |
| B5-7 | FR5-4 choose avoidance moves; WU p9 options include reverse and reattempt | Turn-and-box bypass (S5-U35). Planner can choose REVERSE (`MAX_RETRIES` 2). Until 23 Nov S5 could only request forward and turn moves, so REVERSE meant NO PATH at once; since 23 Nov S5 backs off `REVERSE_CM` (15 cm) straight back (IF-10 reverse bit, S2 `MOTION_PROFILE_REVERSE`, heading held), then scans again (an offset box had read as blocked on both sides) | Matches | #39, #31 |
| B5-8 | FR5-5 / NFR5-4 reacquire the line within 50 cm and hand back | Measured 23 Nov in `integrated`: S5-U42 9 of 9 at about 30 cm from the start of the return leg, and the robot followed the original line each time (S5-U41, INT-10) | Matches | #31 (S5-U41, U42), #35 |
| B5-9 | W6 A5-1: only objects inside the robot's corridor count | Corridor filter exists, with `ROBOT_WIDTH_MM` 150 and `PASS_MARGIN_MM` 50 left unmeasured: decided 22 Nov by the S5 owner that the track will be open, so there is no corridor to calibrate (#45 closed, not planned) | Accepted (owner) | — |
| B5-10 | W6 A5-2 and PT next actions: ignore readings while tilted, hold after tilt clears, check distance rate of change | Ignoring readings is implemented: `s5_detect_allowed()` (`task_obstacle.c`, since `a942e14`, 7 Nov) skips detection while `subsystem4_get_latest()` reports `IMU_MOTION_HUMP` or `IMU_MOTION_TILT`. No hold after the tilt clears and no rate-of-change check. Decided 22 Nov (#44) not to test or extend it: the hump is too small to matter, and the track has none (23 Nov). Corrected 23 Nov: this row and summary row 9 said S5 never read the IMU | Partial | #44 |
| B5-11 | W6 statechart 5: CALIBRATE self-test; SAFE_STOP faults CAL_FAIL, NO_PATH, LINE_LOST sent to telemetry | Self-test done (#47). CAL_FAIL, NO_PATH and LINE_LOST are sent in a 3-bit telemetry field at bits 8–10 (23 Nov, #46). The battery field was removed to make room (nothing measured it); the barcode counters stay 16 bits. Seen on the robot 23 Nov (INT-16): each fault read back from the broker as its own bit | Changed | #46, #47 |
| B5-12 | PT gap check: one distance representation (9-bit vs 16-bit in Version B) | Decided 20 Nov: 9-bit cm on the wire (IF-05), mm inside S5; no 16-bit distance exists in the code | Matches | #54 (`DECISIONS.md`) |
| B5-13 | WU p3: minimise deviation from the route | The planner chooses the smaller feasible shift. Commit `30a2100` reduced the planned extra lateral margin from 120 to 60 mm. Actual detour length remains unmeasured. | Partial | #62 |

## Hardware and pins

The pin plan mostly holds; the IR sensor roles changed, and the agreed pin record is in `TEAM.md`. Encoder and motor pins were not re-checked for this register.

| ID | Week 6 / PT plan | Current code | Type | Follow-up |
| --- | --- | --- | --- | --- |
| H1 | W6 §2: HC-SR04 powered at 3.3 V; PT gap: it needs 5 V and the echo needs a level shifter ("Partially fixed") | Driver is written for the HC-SR04P, the 3.3 V-capable variant | Changed | #69 |
| H2 | W6 §2: IR D0 on GP7 (left), GP27 (centre), GP0 (right); A0 outputs wired but unused | GP28 used as the offset barcode scanner, GP27 and GP0 steer; GP7 not used in SIDE layout | Changed | #7 |
| H3 | Template: UART crash report available | `CRASH_REPORT=0` is required because GP0 is an IR input | Changed | Note in the README build section |
| H4 | W6 §2: ultrasonic TRIG/ECHO on GP16/GP17, servo on GP14 (Robo Pico servo header), IMU I2C0 on GP4/GP5 | Same pins in code | Matches | None |
| H5 | Template rule: claim every pin before wiring | [TEAM.md](TEAM.md#2-pins) records the pin ownership. | Documented | #10 |

## Process, testing and quality

Test logs contain evidence at all three levels. The resource-efficiency report is still open (#61). Barr-C conformance was reviewed on 24 Nov (P3).

| ID | Plan (source) | Current state | Type | Follow-up |
| --- | --- | --- | --- | --- |
| P1 | Template: every test result has a commit and saved evidence | The subsystem, integration and system logs record results. Some manual runs lack the flashed commit or raw capture. | Partial | [Project status](STATUS.md) |
| P2 | PT: test packed interfaces before integration | Host suite (`testing/run_host_tests.sh`) passes 12 of 12 on branch `s1/barcode-queue` (6 all_test-only tests retired, #9). The 2 tests for lost code were resolved (#3): `s1/barcode_queue` now tests the reimplemented queue; `int/steering_ramp` dropped its terrain-boost and barcode slow-down checks (not requirements) and tests the current PWM ramp | Partial | #5, #9 |
| P3 | WU p1: conform to the Barr C coding standard | Reviewed 24 Nov ([BARR_C_REVIEW.md](BARR_C_REVIEW.md)): formatting, braces, final else / switch default, naming (S1 to S5) and `-Wall -Wextra` (0 warnings) done, firmware image unchanged; deviations D1 to D7 agreed (`DECISIONS.md`); 13 `continue`, 17 function-like macros and 48 long lines still open | Matches (with agreed deviations) | #60 (closed) |
| P4 | WU p1, p11: measure resource efficiency before and after packing | Individual build measurements exist, including the [S1 binary change](testing/subsystem1/BINARY_CHANGE.md) and S5-U10. The team report is still open. | Partial | #61 |
| P5 | WU p1: the car operates without remote control | MQTT START begins the run (#48); PAUSE and RESUME exist as manual overrides; the instructor's MQTT map is required. Navigation itself is autonomous: the demo must not rely on manual commands while driving | Matches | Team confirmed 21 Nov that a START command fits this wording (#48) |
| P6 | PT: one shared build process | `build.sh` uses relative repo paths with `PICO_SDK_PATH` supplied by the environment. Build products are ignored. | Implemented | #11 |

## Open decisions

Decision history and the remaining hardware check are listed below. See [DECISIONS.md](DECISIONS.md) for the rationale.

- [x] Final firmware: `integrated` (the Week 6 architecture) or `all_test` (A4, #4). Decided 13 Nov: integrated is final; all_test is a test harness only
- [x] Obstacle detection at 20 cm or back to the required 30 cm (B5-2, #64). Decided 20 Nov: keep 20 cm, if S5-U48 shows a rest ≥ 10 cm away
- [x] Update FR5-1 to the 50 to 130 deg scan, or restore 30 to 150 deg (B5-1, #65). Decided 20 Nov: FR5-1 updated to 50 to 130 deg
- [x] Record why S5 is now the lowest-priority task and the Wi-Fi service the highest (A1, B1-6, #66). Decided 20 Nov: keep; S1-U17 and S2-U12 recorded zero late steps on 22 Nov
- [x] Tilt: stop the robot, or report only as `TRIBUNAL_STATUS.md` says (B4-6, #67). Decided 20 Nov: report only
- [x] Line loss: stop and wait, or actively search as in statechart 3 (B3-2, #68). Decided 20 Nov: stop and wait
- [x] Hump height after the descent: keep the climb height or return to 0 (#5). Decided 20 Nov: keep the climb height
- [x] Start command and Idle/Ready state, or update the system statechart (A5, #48). Decided 20 Nov: add START. The integrated robot test passed on 21 Nov.
- [x] Agree every numeric threshold, including NFR2 tolerances (B2-6, #6). Agreed 20 Nov (`TEST_PLAN.md`)
- [ ] Confirm the ultrasonic part is the HC-SR04P (H1, #69)

Decisions and their evidence feed the Tribunal 2 evidence pack (#63).
