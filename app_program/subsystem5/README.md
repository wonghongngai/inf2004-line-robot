# Buddy 5: ultrasonic scanning, obstacle profiling, avoidance (subsystem 5)

Owner: Buddy 5 (Student ID 2500434). Code: `app_program/subsystem5/`.
Tests: `testing/subsystem5/`. Deliverable for #59.

S5 watches the path ahead with an HC-SR04P ultrasonic sensor on an SG90
servo. When something is in the way it asks the robot to stop, scans the
obstacle, works out where it is and how wide it is, picks a side, drives
round it and finds the line again, then hands control back to line
following. If it can't do that safely, it stops the robot for good and
reports why.

**S5 never drives the motors.** It only posts requests to the Vehicle
Controller (VC, in Buddy 2's `robot_control.c`), which runs each move and
replies (IF-10 / IF-11 below).

This page describes the code as it is on `main` (`ea03e1a`, 23 Nov 2026).
Where it differs from the Week 6 design, see
[Changes from the Week 6 design](#changes-from-the-week-6-design).

## Files

| File | What it does |
|---|---|
| `subsystem5.h` | Public interface: the request word, reply bits, obstacle word, fault bits |
| `subsystem5.c` | `subsystem5_init()` creates `g_flg_s5_evt`; `subsystem5_start()` starts `obstacle_task` |
| `task_obstacle.c` | `obstacle_task`: the state machine, the scans, the bypass and the back-off |
| `obstacle_logic.c/.h` | Pure functions with no hardware and no kernel calls: filter, scan window, median, profile, plan, bypass steps, packing, self-test check. Host-tested |
| `s5_executor.c/.h` | The VC side of the hand-shake, shared by `integrated` and `all_test`. The controller passes in its own drive functions |
| `drv_ultrasonic.c/.h` | HC-SR04P driver, with the echo timed in hardware by a PWM slice |
| `drv_servo.c/.h` | SG90 driver: 50 Hz PWM, calibrated angle, smooth glide |
| `drv_rp2040.c/.h` | 1 MHz timer, short delays, releasing the PWM/TIMER resets |
| `subsystem5_test.c` | On-target and host checks (`TEST_CHECK("S5-Uxx", …)`) |

## Hardware and pins

| Pin / resource | Use |
|---|---|
| GP16 | HC-SR04P TRIG (SIO output, Grove 4) |
| GP17 | HC-SR04P ECHO (PWM slice 0 channel B, gated count, Grove 4) |
| GP14 | SG90 servo signal (PWM slice 7A, Robo Pico servo port 4) |
| PWM slices 0 and 7 | Owned by S5 (TEAM.md section 3) |

- **HC-SR04P, not HC-SR04.** It runs from the 3.3 V Grove rail, so ECHO is
  already 3.3 V and needs no level shifter.
- **Echo timing.** Slice 0 counts at 1 MHz only while ECHO is high
  (`DIVMODE` = B high, `DIV` 125). The task fires a ~12 µs trigger, sleeps
  for the listening window (20 ms for scans, 30 ms for the full 4 m), then
  reads the counter. No interrupt and no busy-wait, so the timing is not
  affected by other tasks, and the encoders keep the GPIO IRQ to
  themselves.
- **Distance.** `d_mm = echo_us × 343 / 2000`, integers only (the RP2040
  has no FPU). Valid range is 20–4000 mm. Anything else, or no echo, is
  invalid (`ultrasonic_classify()`, NFR5-5).
- **Servo.** 1 MHz, `TOP` 19999 (50 Hz). Calibrated across 45° right to
  45° left (1020–2040 µs), with 90° straight ahead. Angles are limited to
  **50–130°**, because the sensor holder fouls the chassis further out.
  Moves glide with a speed ramp (60 → 80 °/s, 100 ms ramp, 60 ms settle
  before a ping), so the swing doesn't jolt the robot.
- The kernel port holds PWM and TIMER in reset. `drv_rp2040.c` releases
  them by clearing reset bits only (`clr_w`), which is safe from both
  cores.

## Task and kernel objects

| Item | Value |
|---|---|
| Task | `obstacle_task`, priority 20, any core (TEAM.md section 4) |
| Event flag | `g_flg_s5_evt`, created and owned by S5, `TA_WSGL` (only S5 waits) |
| Shared words | `g_s5_request`, `g_s5_obstacle`, `g_s5_faults`: one writer (S5), aligned 32-bit stores |

S5 is the **lowest-priority** application task, not the highest as in
Week 6 (`DECISIONS.md` #66). Its slow work is servo travel and 60 ms echo
cycles. Safety comes from the VC, which runs at S2's priority and applies
S5's requests in its own control step.

## How it works

### State machine (`task_obstacle.c`)

```
boot (HOLD) ──self-test pass──▶ MONITORING ──2 of 3 < 20 cm──▶ HOLDING
     │ fail: CAL_FAIL                ▲                            │ STOPPED
     ▼                               │ RESUME                     ▼
 SAFE_STOP ◀── NO_PATH ───────────── │ ◀──── CONTINUE ────── SCANNING ◀──┐
 (reboot)  ◀── LINE_LOST / NO_PATH ─ BYPASSING ◀── PASS L/R ──┤         │
           ◀── VC didn't stop (HOLDING)                        │ REVERSE │
           ◀── back-off failed ─────────────── REVERSING ◀─────┘ ────────┘
                                               (back 15 cm, then rescan)
```

SCANNING gives STOP (→ SAFE_STOP with `NO_PATH`) once `MAX_RETRIES`
back-offs have been used.

| State | What S5 does |
|---|---|
| Start-up | Posts **HOLD** straight away, so the VC can't arm while the servo makes its first swing (the swing shakes the chassis). Waits for S4's IMU calibration, homes the servo, then runs the self-test |
| Self-test (#47) | 5 pings, 100 ms apart, with the servo forward. A working sensor always raises ECHO, even facing open space, so at least 2 pings must show a raw echo time > 0. If not: `CAL_FAIL` → SAFE_STOP, and the wheels never turn |
| MONITORING | Servo forward. One ping every ~70–80 ms while the VC reports `S5_EVT_FOLLOWING` and S4 doesn't report a hump or tilt. An obstacle is confirmed when **2 of the last 3** valid readings are under `D_DETECT_MM` (200 mm). Invalid readings never count as close |
| HOLDING | Posts HOLD and waits up to 1.5 s for `S5_EVT_STOPPED`. No reply → SAFE_STOP |
| SCANNING | Coarse scan, fine scan, profile, plan (below). Publishes the profile for telemetry |
| BYPASSING | Runs the bypass, one request at a time |
| REVERSING | Backs off `REVERSE_CM` (15 cm) straight back, heading held, then scans again |
| SAFE_STOP | Posts SAFE_STOP, parks the servo and sleeps. Only a reboot re-arms it |

After a RESUME, S5 ignores detections for 1 s and clears its reading
history, so the box it has just passed can't trigger it again.

### Scans (FR5-1, FR5-2)

- **Coarse:** jump to 50, 70, 90, 110, 130° and ping at each (5 angles).
- **Fine:** glide smoothly from one coarse step before the first hit to one
  step after the last, pinging every 5° (for example, hits at 90 and 110 →
  70–130°). This measures the edges. A hit is a valid reading under
  `D_PROFILE_MM` (600 mm). No hits means a false alarm, so the fine scan is
  skipped.
- Pings are spaced at least 60 ms apart, including across passes (sensor
  datasheet). The fine scan is smoothed with a median of 3, applied only
  where all three neighbours are valid, so no reading is ever invented.

### Profile (FR5-3, `obstacle_profile()`)

Positions are taken from the sensor: x forward, y left (negative = right).

- **Box:** the run of neighbouring readings within `NEAR_BAND_MM` (150 mm)
  of the closest one. A farther surface seen past the edge isn't part of
  it.
- **Closest point:** the distance and angle of the nearest reading.
- **Width:** the lateral span, corrected for the beam cone.
- **Centre:** the lateral centre of the box.
- **Clearance left / right:** the gap beside each edge. A reading outside
  the box only narrows a clearance if a neighbouring angle confirms it
  (within `CLEAR_CONFIRM_MM`), so one stray echo can't decide the side.
  `OPEN` means the scan looked past that edge and saw nothing. `UNSEEN`
  means the box ran on to the scan limit on that side, so the space is
  unknown.

### Plan (FR5-4, `obstacle_plan()`)

- Box outside the robot's corridor (half the robot's width + margin) →
  **CONTINUE**.
- Otherwise, work out the sideways shift needed for each side and take the
  **feasible side with the smaller shift** (the smaller detour). Ties go to
  the larger clearance. A side counts as feasible if its clearance is at
  least `PASS_GAP_MM` (robot width + 2 × margin). An `UNSEEN` side is used
  only when neither side was seen clear.
- Neither side feasible → **REVERSE**, up to `MAX_RETRIES` (2) times, then
  **STOP** (`NO_PATH`).
- Bad input → STOP.

### Bypass and line recovery (FR5-4, FR5-5, NFR5-4)

This is a turn-and-box path of 90° turns and straight legs, shown here for
a left pass (a right pass is the mirror image):

| Step | Move |
|---|---|
| T1 | Pivot left 90° |
| F1 | Forward the plan's shift + `LOOK_SIDE_EXTRA_MM` (120 mm), limited to 10–40 cm |
| T2 | Pivot right 90°, so the robot is parallel to its original path with the box on its right |
| F2 | **Looked pass:** drive on in 10 cm steps. After each step, **spin on the spot** 90° towards the box, take 3 readings square on, then spin back. Once the box has been seen and a later look is clear, drive on far enough for the tail and the next turn to clear it. Maximum 80 cm |
| T3 | Pivot right 90°, then look ahead. If the box is still in the way, turn back, go 15 cm further and try again (up to 5 times) |
| F3 | Forward until **both** IR sensors have crossed the line, at most `LINE_SEEK_MAX_CM` (50 cm). No line → `LINE_LOST` → SAFE_STOP |
| F4 | Forward 4 cm more, so the wheels are on the line rather than short of it |
| T4 | Pivot left until an IR sensor sees the line again (at most 120°), then **RESUME** |

Why each part is there (from the robot runs; details are in the comments
in `obstacle_logic.h`):

- **Spinning to look (23 Nov).** The earlier pass turned the servo 40° to
  the side and pinged while driving. On a long box face the sound bounced
  away, so the box read as clear while the robot was still beside it, and
  the robot hit it (8 Nov). Square on, the face echoes straight back. A
  spin turns about the robot's centre, so the robot ends where it started.
- **F1 + 12 cm.** On centred runs the box face was 11–17 cm from the
  robot's centre line, but a spin's front corners reach about 11 cm, so
  the robot needs to be further out.
- **F3 waits for both sensors and F4 overshoots.** The IR sensors sit
  ahead of the wheels. Stopping on the first sensor left the robot beside
  the line, 3–5 cm off it (7 Nov).
- **T4 ends on a sensor, not after a fixed 90°.** A fixed turn swept both
  sensors back across the line, and the follower then drifted off.
- **Back-off (23 Nov).** Seen from one close spot, an offset box's sloping
  face read as a second object beside it, so both sides looked blocked.
  Backing off 15 cm and scanning again gives a cleaner view.

## Interfaces

All of these are listed in `TEAM.md` section 5.

### IF-10: S5 → VC request word (`subsystem5_get_request()`)

A packed 32-bit word with a single writer, published with one aligned
store. The VC reads it every control step. A new `seq` means a new
request, which the VC acts on once.

| Bits | Field | Values |
|---|---|---|
| 0–2 | kind | NONE, HOLD, MOVE, RESUME, SAFE_STOP. **5–7 reserved = invalid** |
| 3–4 | move | FWD, TURN_L, TURN_R. **3 reserved = invalid** |
| 5 | stop_on_line | MOVE only: stop when an IR sensor sees black |
| 6–9 | seq | 0–15, changes on every request |
| 10–19 | value | cm (FWD) or degrees (TURN), 0–1023 |
| 20–27 | unused, 0 | must be 0 |
| 28 | line_all | with stop_on_line: wait for every steering sensor |
| 29 | spin | TURN only: turn on the spot |
| 30 | reverse | FWD only: drive backwards, heading held |
| 31 | unused, 0 | must be 0 |

`s5_request_unpack()` rejects reserved values, unused bits that are set,
and spin or reverse on the wrong move. The VC must then stop (the tribunal
fix for corrupted commands, checked by INT-13 and INT-15).

### IF-11: VC → S5 replies (`g_flg_s5_evt`)

| Bit | Meaning |
|---|---|
| `S5_EVT_FOLLOWING` | VC armed and following the line (S5 only watches for obstacles then) |
| `S5_EVT_STOPPED` | HOLD done, robot at rest |
| `S5_EVT_MOTION_DONE` | MOVE finished |
| `S5_EVT_LINE_FOUND` | MOVE ended early on the line |
| `S5_EVT_MOVE_FAILED` | MOVE rejected or timed out |

The VC is the only producer. S5 clears the reply bits it is about to wait
for **before** posting each request, so an old reply can't answer a new
one.

### IF-14: obstacle word for telemetry (`subsystem5_get_obstacle()`)

| Bits | Field |
|---|---|
| 0–8 | closest-point distance, cm, 0–400 |
| 9–16 | servo angle of the closest point, 0–180° |
| 17–25 | beam-corrected width, cm, 0–400 |
| 26–31 | unused, must be 0 |

All zero until the first scan that finds something. A false alarm keeps
the previous result. The VC copies it into the IF-05 telemetry packet
(#40). It is 9-bit cm on the wire and mm everywhere inside S5
(`DECISIONS.md` #54).

### Fault bits (`subsystem5_get_faults()`, #46)

`CAL_FAIL` (bit 0), `NO_PATH` (bit 1), `LINE_LOST` (bit 2). S5 sets a bit
just before entering SAFE_STOP and never clears it. Telemetry sends the
bits in a 3-bit field (S1's `S1_FIELD_S5_FAULTS`).

### IF-08: S4 → S5 (`subsystem4_get_latest()`)

S5 waits for S4's IMU calibration before its first servo swing, and it
doesn't count forward readings while S4 reports `IMU_MOTION_HUMP` or
`IMU_MOTION_TILT` (A5-2).

## Change Requirement 1: minimum-bit data

Every field crossing a task boundary is packed with explicit shifts and
masks, never with C bit-fields, whose layout depends on the compiler.
Each word has one writer and is published with one aligned 32-bit store,
which is atomic on the M0+, so a reader on the other core never sees half
a value.

| Data | Before (struct) | After (packed) |
|---|---|---|
| S5 → VC request | 16 bytes | 4 bytes (IF-10) |
| Obstacle result | 6 bytes | 4 bytes, 26 bits used (IF-14) |
| Faults | — | 3 bits |

Measured in S5-U10 (`testing/subsystem5/logs/2026-11-22_S5-U10_sizes.txt`,
`a063f4b`). That log also has each S5 object's `size` output.

## Build and test

Run from the repo root (see the top-level README for `PICO_SDK_PATH`):

| Command | What you get |
|---|---|
| `sh build.sh integrated` | The full robot. S5 runs as described above |
| `sh build.sh integrated S5_BENCH=1` | S5 keeps HOLD posted (the wheels never turn) and prints every ping with its status and raw echo time. Servo forward. For S5-U02 and S5-U43 |
| `sh build.sh integrated S5_BENCH=2` | As above, and a confirmed obstacle also runs the coarse + fine scan, profile and plan, which are printed. For S5-U07 |
| `sh build.sh integrated TEST=1` | On-target self-tests, including S5's |
| `sh testing/run_host_tests.sh s5/` | The obstacle logic on the PC (S5-U45: filter, window, profile, planner, bypass, packing, self-test) |

Builds without a controller (no `integrated` or `all_test`) have nobody to
answer S5's requests. In those builds S5 only scans and prints the plan.

Results are in [`testing/subsystem5/TEST_LOG.md`](../../testing/subsystem5/TEST_LOG.md)
and the saved logs in `testing/subsystem5/logs/`.

### Requirement status (23 Nov 2026)

| Req | Result | Evidence |
|---|---|---|
| FR5-1 coarse scan | 5 angles, 50–130° (changed, see below) | S5-U04, U11, U22 (servo angles) |
| FR5-2 fine scan | 5° glide over the hit window | S5-U21, U28 |
| FR5-3 profile | Closest point and centre correct; **flat boxes under-read on width** | S5-U07 (owner decision), U31–U34, U38 |
| FR5-4 plan | Left / right / reverse / no-path | S5-U08 (12/12), U35 |
| FR5-5 / NFR5-4 recover | **9/9** reacquired at about 30 cm from the start of the return leg | S5-U41, U42, INT-10 (23 Nov) |
| NFR5-1 detect and stop | **9/9**: trigger at 183–199 mm, at rest 177–190 mm from the box | S5-U48 (23 Nov) |
| NFR5-2 coarse ≤ 600 ms | 570 ms, measured without the sensor on the horn | S5-U05 (28 Oct) |
| NFR5-3 ≤ 2 s | **Dropped**: bench shows 4.6–5.7 s | S5-U06 retired |
| NFR5-5 invalid readings | Never the sole basis of a decision | S5-U03, U13, U43 |
| NFR5-6 accuracy | > 90 % within ±5 cm at 5–60 cm (rule changed) | S5-U02 (22 Nov) |
| Self-test / CAL_FAIL | 9 boots; unplugged sensor → SAFE STOP | S5-U46, U47 |
| Faults in telemetry | Each bit read back from the broker | INT-16 (23 Nov) |

## Changes from the Week 6 design

Full entries are in [`DEVIATIONS.md`](../../DEVIATIONS.md) (B5-x, A1, A3,
M3) and [`DECISIONS.md`](../../DECISIONS.md).

| Week 6 / brief | Now | Why | Ref |
|---|---|---|---|
| Coarse scan 30–150° | 50–130° in 20° steps | The sensor holder can't reach 30° or 150° (5 Nov). FR5-1 updated | B5-1, #65 |
| Detect at ≥ 30 cm | 20 cm | The bypass legs are tuned for it, and the robot still rests ≥ 10 cm away (S5-U48 9/9). NFR5-1 changed | B5-2, #64 |
| Detection → action ≤ 2 s | Dropped | A smooth fine scan was chosen over the time budget (5 Nov) | B5-4 |
| Distance ±2 cm over 5–100 cm | > 90 % within ±5 cm over 5–60 cm | ±2 cm isn't met (5 cm spikes, ~2 cm short at 45 cm). The track doesn't need 100 cm | B5-5 |
| Obstacle task highest priority | Lowest (20) | S5 only requests moves, and safety comes through the VC | A1, #66 |
| `obstacle_q`, `robot_events` group | Packed request word + `g_flg_s5_evt` | One writer per word, no shared event group | A3 |
| 5 V sensor + level shifter | HC-SR04P on 3.3 V | No shifter needed | — |
| Echo ISR | PWM-gated hardware count | No interrupt, and no sharing the GPIO IRQ with the encoders | — |
| Diagonal bypass (turn 45°, …) | Turn-and-box with 90° turns and a looked pass | Easier to tune on encoders. The spin-look fixes false "clear" on long boxes | B5-7, B5-8 |
| Reverse and reattempt | Back off 15 cm, rescan, at most 2 times | S2 reverse moves added 23 Nov | B5-7, #39 |
| Robot corridor measured (A5-1) | `ROBOT_WIDTH_MM` 150, `PASS_MARGIN_MM` 50, not measured | The track is open, so there is nothing to calibrate against | B5-9, #45 |
| Hump test S5-U44 | Dropped | The hump is too small to matter, and the track now has none | #44 |
| Distance width (PT: 9 vs 16 bit) | 9-bit cm on the wire, mm inside | One representation | B5-12, #54 |

## Known limits and open items

- **Detour length not measured (#62).** The planner picks the smaller
  shift, but no run has measured how far the robot leaves the line.
  Measure it in the SYS-04 runs.
- **Width of flat boxes under-reads** (160 mm read as 35–100 mm, B5-6a).
  The bypass covers for this by looking at the box during F2, not trusting
  the width.
- **The reverse back-off hasn't run on the robot.** No run needed it. It
  is host-tested (INT-13).
- **To run on the Pico:** S5-U09 with the spin and reverse bits, and
  S5-U46 (`make TEST=1`).
- **NFR5-2** was last measured without the sensor on the horn. Retest it.
- The bypass lengths marked `[VERIFY]` in `obstacle_logic.h` were tuned on
  one box and one floor. A different box or surface may need retuning.
