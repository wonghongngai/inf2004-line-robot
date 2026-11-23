# Deviations from the original plan

As of 2026-11-13, with the 20 Nov decisions. A copy of the team doc "Group 17 Robot: Deviations from the
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

68 deviations are recorded below: 3 instructor-mandated changes, 17 team design changes and 47 gaps (missing, partial, unverified or to check). The ten that most affect marks or the demo:

| # | Deviation | Type | Fix |
| --- | --- | --- | --- |
| 1 | Barcode letter-to-movement map: no longer hard-coded; both builds apply the MQTT map and go straight until it arrives; telemetry bit 86 reports it (21 Nov, host-tested). A–Z on the robot not tested by decision (21 Nov, `DECISIONS.md`): only A was scanned on the robot | Partial | #38 |
| 2 | IMU collision detection removed and the lift-stop built (#53, 21 Nov); host-tested; S4-U12 passed on the robot 21 Nov | Mandated | #53 |
| 3 | Obstacle detection distance is 20 cm, NFR5-1 requires at least 30 cm | Changed, decided 20 Nov: keep 20 cm | #64 (`DECISIONS.md`); S5-U48 in #31 |
| 4 | All recent tuning lives in `all_test`; the `integrated` firmware (the Week 6 architecture) is untested on the robot | Changed | Decided 13 Nov: integrated is final (#4); port the tuning (#8) |
| 5 | Task priorities inverted: Week 6 made S5 the highest, the code makes it the lowest (20) | Changed, decided 20 Nov: keep | #66 (`DECISIONS.md`); S1-U17 PASS 22 Nov (#15), S2-U12 still to record (#19) |
| 6 | No reverse motion, though WU requires `moveBackward` and the S5 planner can choose REVERSE | Missing | #39 |
| 7 | No IMU motion classes or turn rate (FR4-3, FR4-4; still assessed) | Missing | #42, #43 |
| 8 | Coarse scan uses 50/70/90/110/130 deg, FR5-1 specifies 30/60/90/120/150 deg | Changed, decided 20 Nov: FR5-1 updated | #65 (`DECISIONS.md`) |
| 9 | S5 never reads the IMU, so the A5-2 tilt suppression is not implemented | Missing | #44 |
| 10 | Most requirements have no recorded test result; S5-U02 and S5-U06 last failed | Unverified | #14 to #37 |

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
| A3 | W6 §4 objects: `line_nav_q`, `imu_data_q`, `obstacle_q`, `motion_cmd_q`, `motion_status_q`, `telemetry_q`, `cmd_q`, one `robot_events` event group | S3 message buffer + snapshot API; S4 snapshot under a mutex (`subsystem4_get_latest`), no queue; S5 packed 32-bit request word + `flg_s5_evt` (IF-10/IF-11), no `obstacle_q`; `mbf_motion_cmd` + STOP flag; motion status read through a getter; no shared event group | Changed | Interfaces are listed as IF-05 to IF-11 in `TEAM.md`, but sections 1 and 2 are still template rows (#10) |
| A4 | W6: one integrated system coordinated by the VC | 7 firmware modes. All tuning since about 4 Nov (turns, straight driving, avoidance) is in `all_test`, which skips the subsystem start-up path and replaces the VC with its own control loop. `integrated` UF2 last built 8 Nov and never tested on the robot | Changed | Decided 13 Nov (#4, closed): integrated is final. Port the tuning (#8) |
| A5 | W6 §5 system statechart: Initialising, Idle/Ready waits for a start command, Navigating, Emergency Stop resumes when safe or after a user reset | Waits in Ready after initialisation (5 s, IMU calibrated, S5 hold released) until an MQTT START (code 5); `S2_AUTO_START=1` restores auto-start for a demo without Wi-Fi; line stops re-arm automatically; IMU faults need a reboot | Matches (host-tested; bench test pending) | #48 |
| A6 | PT gap check (Fix): each event bit has one producer, a clear/latch rule and a priority | Event-bit table drafted in `TEAM.md` §5 (20 Nov); no shared event group exists | Partial | #50 |
| A7 | PT gap check (Fix): invalid packed commands are rejected and the robot stays stopped, with an integration test | In `integrated`, MQTT commands reach the VC only as S1's 1-byte packets. S1 rejects wrong lengths, reserved codes and reserved bits before its command buffer. The VC drives `motion_control.c` directly; the S2 packed command (`motion_protocol.h`) has no producer there. INT-15 PASS 23 Nov on the PC (`int/bad_commands`: real S1 MQTT callbacks → VC → motion controller) and on the robot (`integrated`: 26 bad payloads in Ready, following and paused, no effect, all counted invalid). The robot keeps its state; stopped stays stopped | Matches | #51 (closes on merge) |
| A8 | PT gap check (Fix): command ownership by mode, safety pre-empts avoidance, avoidance pre-empts line following | Implemented in `robot_control.c` (IMU checks, then S5 `avoid_step`, then line); host test stubs S5 as idle | Unverified | #20 (hand-over host test), #35 (INT-10) |

## Buddy 1: Wi-Fi, MQTT, telemetry

The communication stack is built as designed, but none of FR1 or NFR1 has been tested on the Pico, and the Wi-Fi service runs at a higher priority than the control tasks.

| ID | Requirement (source) | Current code | Type | Follow-up |
| --- | --- | --- | --- | --- |
| B1-1 | FR1-1 Wi-Fi + MQTT connection (W6) | CYW43 + lwIP + MQTT through the kernel Wi-Fi service; PT noted connectivity "not yet tested" | Verified 22 Nov | #15: S1-U11 5/5 joins in ~4.1 s (22 Nov), S1-U12 PASS (21 Nov) |
| B1-2 | FR1-2 publish telemetry; WU p5 examples include obstacle scan results, encoder counts, motor speed | 19-byte packet, 1 s period, published by both `all_test` and `integrated` (VC, #72, 21 Nov; host-tested). Obstacle distance, scan angle and width fields are still 0. Encoder counts added 23 Nov (#41): left/right raw counts as their low 16 bits (wrapping), host-tested and checked on the robot (S1-U12, S1-U21); motor speed is the existing average speed field | Partial | #40, #41 |
| B1-3 | FR1-3 receive commands (W6) | The integrated VC handles STATUS, PAUSE, RESUME and START and applies the MQTT letter map (#72, #48, 21 Nov; host-tested, not yet on the robot). RESET_STATS and REVERSE are logged as unsupported | Partial | #38, #39 |
| B1-4 | FR1-4 / NFR1-2 reconnect without restart (W6) | Recovery logic in `mqtt_manager.c` (MQTT retry every 3 s) and `cyw43_utk.c` (Wi-Fi rejoin) | Verified 22 Nov | #15: S1-U13 PASS on the robot, telemetry back 2.8 s after a broker restart and 3.1 s after the hotspot; needs PR #102 (S4 core pin, merged into `main` 22 Nov), without which the IMU fault stopped the robot during outages |
| B1-5 | FR1-5 periodic status (W6) | 1-byte heartbeat every 2 s (`MQTT_UTK_HEARTBEAT_MS` = 2000; this row said 5 s until 22 Nov) | Unverified | #15 (seen on the broker during the 22 Nov runs, no saved log) |
| B1-6 | NFR1-3 telemetry must not disturb control; W6 gave comms **medium** priority | Wi-Fi service task priority 4, above motion (14) and line (16) | Changed, decided 20 Nov: keep | #66 (`DECISIONS.md`). Measured 22 Nov (S1-U17 PASS): a 100-command flood left `ctrl late` at 0, and the same boot had 0 late control steps in 66 min with Wi-Fi on, through broker and hotspot outages. S2-U12 (#19) still to record |
| B1-7 | PT gap: define handling of commands during a connection loss | Defined 22 Nov (`app_program/subsystem1/README.md`, `DECISIONS.md`): commands and maps sent while the robot is offline are dropped, never replayed (lwIP always connects with a clean session); every command is safe to repeat; senders check the heartbeat and never queue or retain commands | Verified 22 Nov | #49: S1-U20 PASS on the robot |
| B1-8 | Barcode result events on `<id>/barcode` (`BARCODE_MQTT.md`) | The original code was lost; reimplemented 20 Nov on branch `s1/barcode-queue` (S1 queue, S3 hook, dashboard panel). Payload changed from JSON to a 4-byte binary packet so every topic follows protocol v1. Host tests pass; not yet run on the robot | Changed | #3 (closed), #16 |
| B1-9 | WU deliverables: communication API, telemetry framework, MQTT documentation, telemetry demo | S1 README documents the packet layout and map topic; no demo evidence | Partial | #55 |

## Buddy 2: Motion control

Motor and encoder drivers follow the PT design, but PID control is not tuned, reverse motion was dropped against the brief, and straight-line correction exists only in the all_test build.

| ID | Requirement (source) | Current code | Type | Follow-up |
| --- | --- | --- | --- | --- |
| B2-1 | FR2-1 motors with encoder feedback (W6); PT: count edges in the ISR, no task wake-up per pulse | Implemented as PT decided (`drv_encoder.c`, IRQ counting); not tested on the robot | Unverified | #18 (S2-U05) |
| B2-2 | FR2-2 / NFR2-2 speed and distance from encoders | `wheel_odometry.h`; `CALIBRATION.md`: old calibration "not validated for IRQ counts" | Partial | #18 (S2-U06, S2-U07) |
| B2-3 | WU p6 PID speed control | PID present, gains still 0 pending physical tuning | Missing | #18 (S2-U10), #56 (tuning report) |
| B2-4 | FR2-3 / NFR2-1 straight-line correction | Encoder trim in `all_buddies_test.c` only (`ab62be5`); not in the integrated controller | Partial | #8, #19 (S2-U08) |
| B2-5 | FR2-4 forward, turn, stop; WU p6 required APIs include `moveBackward(distance)` | Commands: FORWARD_CM, TURN_LEFT_DEG, TURN_RIGHT_DEG, STOP, RESET_FAULT, FOLLOW_LINE. "There is intentionally no backwards command" (W6 §8.1 dropped it on AI advice) | Changed, now reversed by team decision | #39 |
| B2-6 | NFR2-1 to NFR2-3 measurable tolerances (W6 gap marked "Fixed") | Thresholds agreed 20 Nov in `TEST_PLAN.md` (S2-U07 to U13); NFR2 wording itself not yet updated | Partial | #6 (`DECISIONS.md`) |
| B2-7 | PT: encoder counter widths decided from measured range (CR1) | Done 23 Nov (#52, S2-U14): widths from the robot logs, status 30 → 12 bytes, "provisional" removed. Other subsystems not yet migrated to the packed status | Partial | #52 (widths); migration open |
| B2-8 | WU deliverables: motion library, PID tuning report, motion accuracy evaluation | Library exists; no report or evaluation | Missing | #56 |

## Buddy 3: Line following and barcode

The decoder is the best-tested part of the project on the PC, but the sensor layout changed from three line sensors to two plus a barcode scanner, and the robot stops on line loss instead of searching.

| ID | Requirement (source) | Current code | Type | Follow-up |
| --- | --- | --- | --- | --- |
| B3-1 | FR3-1 three IR sensors (GP7 left, GP27 centre, GP0 right) all used for line following (W6 §2) | SIDE layout in both builds: GP28 is an offset barcode scanner, only GP27 and GP0 steer. Top-level README still describes the 3-sensor layout | Changed | #7 |
| B3-2 | W6 statechart 3: on line loss, Search Line mode adjusts movement until the line is found | Stop and wait, no search motion. `integrated` steers in the straddle model (#8): C and R sit either side of the tape and both white is the normal reading, so S3's LOST is ignored. Since #23 the VC detects loss itself: a correction of ≥ 150 ms that clears to white with no black within `S2_LINE_LOST_MM` (150 mm, encoders) starts a soft probe toward the side the tape was last under (`S2_LINE_PROBE_MM`, 100 mm); black ends it, none stops the robot; it re-arms once C or R sees the tape and it is held still 1 s, or after PAUSE + RESUME. Host-tested; two false stops on the track (23 Nov, tape still between C and R) led to the probe, which is not yet run on the robot. `LINE_BARCODE_SEARCH_MS` is 0 in both robot builds | Changed, decided 20 Nov: stop and wait; statechart 3 updated (course has no gaps). Straddle VC gets its own loss check (#23, 23 Nov) | #68 (`DECISIONS.md`); S3-U31 in #23 |
| B3-3 | FR3-2 junction detection | Cross-junction from the scanner seeing black for 4.5 narrow-bar widths; host test S3-U25 passes; not tested on the track | Unverified | #23 |
| B3-4 | FR3-3 decode navigation barcodes (W6); prof: all A to Z, both directions | Decoder handles A to Z both ways on the PC (S3-U07); on hardware only A and Z were tried (`208188b`) | Partial | #23 (S3-U32, S3-U35) |
| B3-5 | W6 statechart 3: an invalid barcode is ignored and line tracking continues | Matches; now also planned: reverse and re-scan a failed barcode | Changed (addition) | #39 |
| B3-6 | Sampling: periodic sensor processing (W6) | Period changed from 10 ms to 1 ms on 3 Nov (`b276d54`) for barcode resolution | Changed | Record CPU cost in the resource report (#61) |
| B3-7 | NFR3-1 robust to lighting, track and speed changes; NFR3-2 reliable recognition | No track or lighting tests recorded; on-target S3 unit tests passed 19 Nov (d9c4830; S3-U01 retired) | Unverified | #21, #22, #23 |
| B3-8 | WU deliverables: line-following module, barcode decoder, navigation command interface | Code and S3 README exist; README map section is out of date (says MQTT maps are ignored) | Partial | #57 |

## Buddy 4: IMU, humps, tilt

Calibration, tilt and hump detection exist, but motion classes and turn rate (still assessed) are missing, collision was replaced by a lift-stop (S4-U12 passed 21 Nov), and all thresholds are placeholders.

| ID | Requirement (source) | Current code | Type | Follow-up |
| --- | --- | --- | --- | --- |
| B4-1 | FR4-1 calibrate before monitoring (W6) | 1 s stationary baseline at start; on-target check S4-U03 passed 19 Nov (`d9c4830`); baseline values not yet logged on the bench | Partial | #24 |
| B4-2 | FR4-2 tilt and hump events with peak estimate (W6); WU: measure the highest hump peak | Implemented. The estimator keeps the climb height after the descent (decided 20 Nov, #5; `s4/hump_height` passes). Thresholds are placeholders (`subsystem4/README.md`) | Partial | #25, #27 |
| B4-3 | FR4-3 motion classification (W6); WU p8 events: stationary, accelerating, turning, climbing, descending | Only NORMAL, HUMP, TILT, LIFTED, FAULT safety states | Missing | #42 |
| B4-4 | FR4-3 collision detection (W6) | Removed 21 Nov (#53); the lift check replaces it | Mandated removal (M2) | #53 |
| B4-5 | FR4-4 report turn rate (W6, WU) | No turn rate; the LSM303DLHC has no gyroscope; magnetometer is read but unused | Missing | #43 |
| B4-6 | PT decision: a normal hump must not cause an emergency stop; tilt and hump are informational (`TRIBUNAL_STATUS.md`) | Decided 20 Nov: tilt is report-only. The VC no longer halts on tilt (`robot_control.c`); host test INT-03 passes | Matches (not yet tested on the robot) | INT-11 dropped 22 Nov (#36): the ultrasonic sits above the hump |
| B4-7 | NFR4-1 sample every 20 ms; NFR4-3 cycle within 20 ms | `S4_SAMPLE_PERIOD_MS` back to 20 on 20 Nov, with every sample count halved to keep the same timings (it was 10 from `8dde092`, 18 Nov). At 10 ms, S4-U05 **failed** on target 19 and 20 Nov: cycles take about 12 ms (longest 17 ms with Wi-Fi off, 20 ms with it on). At 20 ms, S4-U05 **passed** on target 20 Nov (0 overruns, longest cycle 19 ms); hump detection not yet re-checked at 20 ms | Unverified | #24 |
| B4-8 | NFR4-2 report a hump within 100 ms | Not measured; the collision part of NFR4-2 is void (M2) | Unverified | #27 |
| B4-9 | CR1 minimum-bit representation | 4-byte `IMU_PACKED_MSG` exists (S4-U06), but `subsystem4/README.md` still calls this the "PRE" version; no before/after sizes | Partial | #25 (S4-U14) |
| B4-10 | WU deliverables: IMU module, hump algorithm, terrain analysis report | No report | Missing | #58 |

## Buddy 5: Ultrasonic scanning and avoidance

The full scan, profile, plan and bypass pipeline exists, but the detection distance and scan angles were changed from FR5-1 and NFR5-1, two NFRs last failed, and both Week 6 assumptions are still open.

| ID | Requirement (source) | Current code | Type | Follow-up |
| --- | --- | --- | --- | --- |
| B5-1 | FR5-1 coarse scan at 30, 60, 90, 120, 150 deg (W6, WU) | 50, 70, 90, 110, 130 deg; scan range narrowed to 50 to 130 deg on 5 Nov because the sensor holder cannot reach 30 or 150 deg. Decided 20 Nov: FR5-1 updated to match. The S5 test log says NFR5-2/NFR5-3 were dropped; they were not, and must be re-measured | Changed | #65 (`DECISIONS.md`); NFR5-2 re-test in #29 |
| B5-2 | NFR5-1 detect at 30 cm or more, rest at least 10 cm away | `D_DETECT_MM` 200 mm since 7 Nov, for the turn-and-box bypass. Decided 20 Nov: keep 20 cm; NFR5-1 becomes detect at 20 cm or more, rest at least 10 cm away. S5-U48 PASS 23 Nov: confirmed by the 200 mm trigger (183–199 mm) and at rest 177–190 mm from the box over 9 runs. Buddy 5 still has to record the original reason | Changed | #64 (`DECISIONS.md`); S5-U48 (#31) |
| B5-3 | FR5-2 return to the region and scan in 5 deg steps | Smooth glide with a ping every 5 deg, window capped by `FINE_BUDGET_POINTS` | Matches | None |
| B5-4 | NFR5-3 detection to action within 2 s | Limit dropped for a smooth fine scan in the 5 Nov scan rework (the `FINE_BUDGET_POINTS` cap is gone); S5-U06 retired 22 Nov by the S5 owner (#29). Last result FAIL, 2299 ms (1 Nov). The bench image prints the time for information | Dropped | — |
| B5-5 | NFR5-6 closest point within 2 cm over 5 to 100 cm | Hardware-timed echo since 27 Oct. Decided 22 Nov by the S5 owner (#29): S5-U02 checks 5 / 15 / 30 / 45 / 60 cm only (100 cm dropped) and passes when > 90 % of readings are within ±5 cm. PASS on that rule; ±2 cm is not met (5 cm spikes, ~2 cm short at 45 cm) | Changed (range and tolerance) | — |
| B5-6 | FR5-3 profile location, width, closest point, side clearances | Implemented; on-target tests S5-U31 to U38 written, no recorded results | Unverified | #28 |
| B5-6a | FR5-3 width of the obstacle | S5-U07 (22 Nov): closest point and centre correct, but flat boxes under-read (160 mm → 35–100 mm); marked done by the S5 owner because the test box has angled sides | Accepted (owner) | — |
| B5-7 | FR5-4 choose avoidance moves; WU p9 options include reverse and reattempt | Turn-and-box bypass (S5-U35). Planner can choose REVERSE (`MAX_RETRIES` 2). Until 23 Nov S5 could only request forward and turn moves, so REVERSE meant NO PATH at once; since 23 Nov S5 backs off `REVERSE_CM` (15 cm) straight back (IF-10 reverse bit, S2 `MOTION_PROFILE_REVERSE`, heading held), then scans again (an offset box had read as blocked on both sides) | Matches | #39, #31 |
| B5-8 | FR5-5 / NFR5-4 reacquire the line within 50 cm and hand back | Measured 23 Nov in `integrated`: S5-U42 9 of 9 at about 30 cm from the start of the return leg, and the robot followed the original line each time (S5-U41, INT-10) | Matches | #31 (S5-U41, U42), #35 |
| B5-9 | W6 A5-1: only objects inside the robot's corridor count | Corridor filter exists, with `ROBOT_WIDTH_MM` 150 and `PASS_MARGIN_MM` 50 left unmeasured: decided 22 Nov by the S5 owner that the track will be open, so there is no corridor to calibrate (#45 closed, not planned) | Accepted (owner) | — |
| B5-10 | W6 A5-2 and PT next actions: ignore readings while tilted, hold after tilt clears, check distance rate of change | S5 never reads the IMU | Missing | #44 |
| B5-11 | W6 statechart 5: CALIBRATE self-test; SAFE_STOP faults CAL_FAIL, NO_PATH, LINE_LOST sent to telemetry | Self-test done (#47). CAL_FAIL, NO_PATH and LINE_LOST are sent in a 3-bit telemetry field at bits 8–10 (23 Nov, #46). The battery field was removed to make room (nothing measured it); the barcode counters stay 16 bits. Seen on the robot 23 Nov (INT-16): each fault read back from the broker as its own bit | Changed | #46, #47 |
| B5-12 | PT gap check: one distance representation (9-bit vs 16-bit in Version B) | Decided 20 Nov: 9-bit cm on the wire (IF-05), mm inside S5; no 16-bit distance exists in the code | Matches | #54 (`DECISIONS.md`) |
| B5-13 | WU p3: minimise deviation from the route | Detour length never measured | Unverified | #62 |

## Hardware and pins

The pin plan mostly holds; the IR sensor roles changed, and the agreed pin record in `TEAM.md` was never filled in. Encoder and motor pins were not re-checked for this register.

| ID | Week 6 / PT plan | Current code | Type | Follow-up |
| --- | --- | --- | --- | --- |
| H1 | W6 §2: HC-SR04 powered at 3.3 V; PT gap: it needs 5 V and the echo needs a level shifter ("Partially fixed") | Driver is written for the HC-SR04P, the 3.3 V-capable variant | Changed | #69 |
| H2 | W6 §2: IR D0 on GP7 (left), GP27 (centre), GP0 (right); A0 outputs wired but unused | GP28 used as the offset barcode scanner, GP27 and GP0 steer; GP7 not used in SIDE layout | Changed | #7 |
| H3 | Template: UART crash report available | `CRASH_REPORT=0` is required because GP0 is an IR input | Changed | Note in the README build section |
| H4 | W6 §2: ultrasonic TRIG/ECHO on GP16/GP17, servo on GP14 (Robo Pico servo header), IMU I2C0 on GP4/GP5 | Same pins in code | Matches | None |
| H5 | Template rule: claim every pin in `TEAM.md` before wiring | `TEAM.md` sections 1 and 2 still hold template rows | Missing | #10 |

## Process, testing and quality

The evidence rules from the template are mostly not followed yet, and two assessed qualities (Barr-C conformance, resource efficiency) have no work recorded.

| ID | Plan (source) | Current state | Type | Follow-up |
| --- | --- | --- | --- | --- |
| P1 | Template `testing/README.md`: every test has a TEST_LOG row with a commit and a saved log | S1, S2, S4 and system logs have no results; their test-case tables still describe template LED tests while the code checks S1-U01 to U10, S2-U01 to U04, S4-U01 to U07; many S5 rows say "console, not saved" | Partial | #14, #17, #21, #24, #28 |
| P2 | PT: test packed interfaces before integration | Host suite (`testing/run_host_tests.sh`) passes 12 of 12 on branch `s1/barcode-queue` (6 all_test-only tests retired, #9). The 2 tests for lost code were resolved (#3): `s1/barcode_queue` now tests the reimplemented queue; `int/steering_ramp` dropped its terrain-boost and barcode slow-down checks (not requirements) and tests the current PWM ramp | Partial | #5, #9 |
| P3 | WU p1: conform to the Barr C coding standard | No review recorded | Missing | #60 |
| P4 | WU p1, p11: resource efficiency is assessed; PT: measure timing before and after packing | No flash, RAM, stack or task-timing figures recorded | Missing | #61 |
| P5 | WU p1: the car operates without remote control | MQTT START begins the run (#48); PAUSE and RESUME exist as manual overrides; the instructor's MQTT map is required. Navigation itself is autonomous: the demo must not rely on manual commands while driving | Matches | Team confirmed 21 Nov that a START command fits this wording (#48) |
| P6 | PT: one shared build process | Build scripts hard-code one teammate's paths; build outputs were committed until 13 Nov (now ignored) | Partial | #11 |

## Open decisions

Each "Changed" row needs either a recorded justification for Tribunal 2 or a return to the original plan. These ten need a team decision first. The reasons for each decision are in [DECISIONS.md](DECISIONS.md):

- [x] Final firmware: `integrated` (the Week 6 architecture) or `all_test` (A4, #4). Decided 13 Nov: integrated is final; all_test is a test harness only
- [x] Obstacle detection at 20 cm or back to the required 30 cm (B5-2, #64). Decided 20 Nov: keep 20 cm, if S5-U48 shows a rest ≥ 10 cm away
- [x] Update FR5-1 to the 50 to 130 deg scan, or restore 30 to 150 deg (B5-1, #65). Decided 20 Nov: FR5-1 updated to 50 to 130 deg
- [x] Record why S5 is now the lowest-priority task and the Wi-Fi service the highest (A1, B1-6, #66). Decided 20 Nov: keep; loop lateness measured 22 Nov (S1-U17: 0 late), S2-U12 still to record
- [x] Tilt: stop the robot, or report only as `TRIBUNAL_STATUS.md` says (B4-6, #67). Decided 20 Nov: report only
- [x] Line loss: stop and wait, or actively search as in statechart 3 (B3-2, #68). Decided 20 Nov: stop and wait
- [x] Hump height after the descent: keep the climb height or return to 0 (#5). Decided 20 Nov: keep the climb height
- [x] Start command and Idle/Ready state, or update the system statechart (A5, #48). Decided 20 Nov: add a START command (to build)
- [x] Agree every numeric threshold, including NFR2 tolerances (B2-6, #6). Agreed 20 Nov (`TEST_PLAN.md`)
- [ ] Confirm the ultrasonic part is the HC-SR04P (H1, #69)

Decisions and their evidence feed the Tribunal 2 evidence pack (#63).
