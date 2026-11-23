# Team and subsystems

This file is the **target design** of the final firmware, `ROBOT_MODE=integrated`
(decided in #4), and **who builds each part**. It describes the robot as it
should be at the demo, not as it is today:

- what is done, with evidence: [STATUS.md](STATUS.md)
- where the code or the plan differs from the Week 6 / PreTri 1 design, and why:
  [DEVIATIONS.md](DEVIATIONS.md)

Where the team has not decided yet, the current value is shown and tagged
**TBD** with its issue. When the issue is decided, update the row here.

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (finished scope) | Name | Student ID | GitHub username | Code | Tests |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | Wi-Fi and MQTT link; telemetry, PC commands, barcode result events, receiving the letter-to-movement map | Kan Ka En | 2503598 | `dunnknow` | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motors, encoders, PID speed control; forward, reverse, turn and stop; the Vehicle Controller (`robot_control.c`) that coordinates every subsystem | Kang Zhi Xian | 2503986 | `2andbeii` | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IR line following and line-loss handling; Code 39 barcodes A–Z in both directions; junctions, acting on the stored letter at the next cross-junction | Chai Sook Hui | 2502871 | `chaisookhui` | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU (LSM303DLHC): calibration, humps and highest-hump height, tilt, lift-stop, motion classes, turn rate. No collision detection | Lew Zhi Xin | 2503689 | `lewzhixin` | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Ultrasonic + servo scanning, obstacle profiling, avoidance planning, line recovery, ignoring humps and the floor while tilted | Wong Hong Ngai | 2500434 | `wonghongngai` | `app_program/subsystem5/` | `testing/subsystem5/` |

### Work allocation

Each buddy owns their subsystem's code, its tests in `testing/subsystemN/`,
and these write-up deliverables:

| # | Deliverables | Issue |
| --- | --- | --- |
| 1 | Communication API, telemetry framework, MQTT documentation, telemetry demo | #55 |
| 2 | Motion library, PID tuning report, motion accuracy evaluation | #56 |
| 3 | Line-following module, barcode decoder, navigation command interface | #57 |
| 4 | IMU module, hump algorithm, terrain analysis report | #58 |
| 5 | Scanning, obstacle profiling, avoidance and recovery | #59 |

Shared by the whole team: Barr-C coding standard review (#60), resource
efficiency report (#61), Tribunal 2 evidence pack (#63), integration tests
(`testing/integration/`).

Work that crosses subsystems has a lead owner:

| Work | Lead | With | Issue |
| --- | --- | --- | --- |
| Connect the integrated firmware to MQTT: telemetry, commands, letter map | S2 (VC) | S1 | #72 |
| Apply the MQTT letter map; remove the hard-coded maps | S3 | S2 | #38 |
| Reverse motion (`moveBackward`), used by S5 and barcode re-scan | S2 | S3, S5 | #39 |
| Obstacle scan results in telemetry | S5 | S1 | #40 |
| Encoder counts and motor speed in telemetry | S1 | S2 | #41 |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin.

| Pin | Owner | Used for |
| --- | --- | --- |
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S3 | right IR, digital input (steering). Needs `CRASH_REPORT=0` |
| GP1 | | free (the final build uses `CRASH_REPORT=0`, so the crash UART does not take it) |
| GP2 | S2 | right encoder A |
| GP3 | S2 | right encoder B |
| GP4 | S4 | I2C0 SDA, LSM303DLHC |
| GP5 | S4 | I2C0 SCL, LSM303DLHC |
| GP6 | S2 | left encoder B |
| GP7 | | free (was the Week 6 left IR; unused in the SIDE layout) |
| GP8 | S2 | left motor forward PWM |
| GP9 | S2 | left motor reverse PWM (#39) |
| GP10 | S2 | right motor forward PWM |
| GP11 | S2 | right motor reverse PWM (#39) |
| GP12 | | |
| GP13 | | |
| GP14 | S5 | SG90 servo PWM (slice 7A, Robo Pico servo port 4) |
| GP15 | | free (the kernel does not start I2C1) |
| GP16 | S5 | HC-SR04P TRIG (SIO output, Grove 4) |
| GP17 | S5 | HC-SR04P ECHO (PWM slice 0B input, gated echo timing, Grove 4) |
| GP18 | | |
| GP19 | | |
| GP20 | | |
| GP21 | | |
| GP22 | S1 | Robo Pico piezo buzzer, PWM slice 3A (Wi-Fi connect beep, `drv_buzzer.c`) |
| GP26 | S2 | left encoder A; ADC pin initialization disabled |
| GP27 | S3 | centre IR, digital input (steering); ADC pin initialization disabled |
| GP28 | S3 | offset barcode scanner IR, digital input; ADC pin initialization disabled |

IR layout: SIDE, as wired on the robot. GP27 (centre) and GP0 (right) steer
and sit **either side of the black line** (C/R = 0/0 on the line; found 21 Nov,
#8: both builds now use this model, `S2_LINE_STRADDLE`),
and GP28 (left) reads barcodes beside the line. Each sensor's digital output
(D0) is wired; the analog outputs (A0) are not connected.

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
| --- | --- | --- |
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
| --- | --- | --- | --- |
| UART0 | unused: the kernel starts its serial driver (`sera`) at boot but, with `CRASH_REPORT=0`, never connects it to a pin. No crash reports in the final firmware | — | — (GP0 is S3's right IR, GP1 is free) |
| UART1 | | | |
| I2C0 | S4 via kernel device `iica` | `imu_task` | GP4/GP5 |
| I2C1 | not started | — | — |
| SPI0 | | | |
| SPI1 | | | |
| ADC | unused: the kernel driver is enabled but every IR input is digital | — | — |
| PWM slices 4, 5 | S2 | `motion_task` | GP8–GP11 |
| PWM slices 0, 7 | S5 | `obstacle` (`obstacle_task`) | GP17 (echo timing), GP14 (servo) |
| IO_IRQ_BANK0 (GPIO interrupts, IRQ 13) | S2 | interrupt handler in `drv_encoder.c` (enabled on the core that calls `drv_encoder_init`) | edge interrupts on GP26/GP6 (left encoder), GP2/GP3 (right encoder) only. Any other GPIO interrupt must go through this handler; ask S2 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:

  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```

- **Send or receive data over the network** through S1's message buffers.
  The MQTT client in `app_program/subsystem1/mqtt_manager.c` runs inside the
  service task: it drains the outgoing buffers with `TMO_POL` (never
  blocking) and publishes them, and fills the incoming ones from the
  subscribed topics. The interfaces are IF-05, IF-06, IF-12 and IF-13 in section 5. It needs
  the `NET=lwip MQTT=1` build (README section 1.1).

## 4. Task priorities

1 is the highest and 32 the lowest (`CNF_MAX_TSKPRI` in `config/config.h`).
1–4 are used by the kernel. Each subsystem gets its own band; change it only
by team agreement. Write down every task you create.

| Subsystem | Priority band |
| --- | --- |
| kernel | 1–4 |
| *(unused)* | 5–11 |
| 1 | 12–13 |
| 2 | 14–15 |
| 3 | 16–17 |
| 4 | 18–19 |
| 5 | 20–21 |

**Decided 20 Nov (#66): keep this order**, although Week 6 made S5 the
highest. S2 and S3 run on core 1 and Wi-Fi on core 0, so Wi-Fi cannot
preempt them. S5 only requests moves, and the VC applies them at S2's
priority. Reasons in [DECISIONS.md](DECISIONS.md). Still to measure:
control-loop lateness with Wi-Fi on (S1-U17, S2-U12).

Tasks in the final firmware, highest priority first:

| Task | Owner | Priority | Core | Role | Source |
| --- | --- | --- | --- | --- | --- |
| initial task | kernel | 1 | — | runs `usermain()` (and the `TEST=1` self-tests), then sleeps | `kernel/inittask/inittask.c` |
| USB console service | kernel | 3 | core 0 | serial console | `lib/libtm/.../usb/usb_console.c` |
| `cyw43` (`cyw43_utk_task`) | kernel | 4 | core 0 | Wi-Fi, lwIP, MQTT; runs S1's MQTT client and services S1's buffers | `lib/libwifi/.../cyw43_utk_kernel.c` |
| — | S1 | — | — | no task of its own: all S1 network code runs in `cyw43` | `subsystem1/` |
| `motion_task` | S2 | 14 | core 1 | PID loop and the Vehicle Controller | `subsystem2/subsystem2.c` |
| `line_service_task` | S3 | 16 | core 1 | IR sampling (1 ms), barcode decoding, junctions; on core 1, away from the radio | `subsystem3/line_service.c` |
| `imu_task` | S4 | 18 | any | IMU sampling (20 ms), terrain and safety state | `subsystem4/subsystem4.c` |
| `obstacle` (`obstacle_task`) | S5 | 20 | any | scan, profile, plan, request avoidance moves | `subsystem5/subsystem5.c` |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

Test harnesses (`all_test`, `line_follow_test`, `barcode_test`, `motor_test`,
`motion`, `integrated_motor_test`; README section 2) add or replace tasks
inside the owning subsystem's band. They are not part of this design.

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID or access function in its own
`subsystemN.h`; the **user** only sends to it or reads it. Agree on the data
format before coding.

The Vehicle Controller (VC, `robot_control.c`, in S2's `motion_task`) is the
one place that reads every subsystem and decides what the robot does: safety
first, then S5 avoidance, then line following.

| ID | From → To | Mechanism | Object ID / function | Data / meaning | Agreed by | Issue |
| --- | --- | --- | --- | --- | --- | --- |
| IF-05 | VC → S1 | message buffer, nonblocking send (owned by S1) | `subsystem1_publish_telemetry()` | 80-bit telemetry packet once a second: state, faults, speed, distance, line sensors, barcode, highest hump, encoder counts, obstacle scan results (layout in `subsystem1/README.md`) | Buddy 1; VC publishes it (#72) | #40, #41 |
| IF-06 | S1 → VC | message buffer, polled with `TMO_POL` (owned by S1) | `subsystem1_receive_command()` | PC commands STATUS, PAUSE, RESUME, RESET_STATS, START | Buddy 1; VC handles STATUS, PAUSE, RESUME, START (#72, #48) | #49 |
| IF-07 | S3 → VC | S3 message buffer and snapshot API | `g_h_line_nav_q` | line positions; barcode letter and junction command; one consumer | integrated build | #38, #68 |
| IF-08 | S4 → VC, S5 | snapshot under S4 mutex | `subsystem4_get_latest()` | IMU calibrated, terrain (hump, tilt, highest-hump height), lift-stop, motion class, turn rate. S5 ignores readings while tilted or on a hump (A5-2) | integrated build | #53, #42, #43, #44, #67 |
| IF-09 | Application → S2 | message buffer, separate STOP flag | `mbf_motion_cmd`, `flg_motion_stop` | `MOTION_COMMAND` (forward, reverse, turn, stop); autonomous profiles reject external moves | motion build | #39 |
| IF-10 | S5 → VC | packed 32-bit request word, single writer, read with `subsystem5_get_request()` (owned by S5) | `subsystem5_get_request()` | `S5_REQUEST`: NONE / HOLD / MOVE{fwd or reverse cm, pivot deg, stop-on-line (first sensor, or with bit 28 `line_all` once every steering sensor has crossed), bit 29 `spin` on a turn = turn on the spot, both wheels counter-rotating, bit 30 `reverse` on a forward leg = drive it backwards (both 23 Nov; set on any other request = reserved)} / RESUME / SAFE_STOP plus a 4-bit seq; reserved values → VC stops (`E_PAR`). S5 posts HOLD at boot and the VC does not arm until S5 releases it (after the servo's start-up swing) | **proposed** by Buddy 5; needs Buddy 2's OK | #39 |
| IF-11 | VC → S5 | event flag (owned by S5) | `flg_s5_evt` | `S5_EVT_FOLLOWING` / `STOPPED` / `MOTION_DONE` / `LINE_FOUND` / `MOVE_FAILED` | **proposed** by Buddy 5; needs Buddy 2's OK | — |
| IF-12 | S1 → VC → S3 | message buffer (owned by S1), then S3 setter | `subsystem1_receive_mapping()`, `line_service_set_mapping()` | 4-letter letter-to-movement map sent from the PC before the demo. Until a map arrives, the robot goes straight at junctions; a junction with no mapped barcode before it also goes straight. No hard-coded map in either build; telemetry bit 86 reports whether a map has been applied | VC and all_test apply it (#72, #38) | #38 |
| IF-13 | S3 → S1 | message buffer (owned by S1), nonblocking send | `subsystem1_publish_barcode()` | barcode result event: frame, valid/invalid, letter, run count; published on `<id>/barcode` as a 4-byte binary packet with QoS 1 (`testing/subsystem1/BARCODE_MQTT.md`) | host-tested (branch `s1/barcode-queue`) | #3, #16 |
| IF-14 | S5 → VC | packed 32-bit obstacle word, single writer, read with `subsystem5_get_obstacle()` (owned by S5) | `subsystem5_get_obstacle()` | `S5_OBSTACLE_INFO`: last scan that found an obstacle: closest-point distance (cm, 0–400), its servo angle (°, 0–180), beam-corrected width (cm, 0–400). All 0 until the first such scan; a scan that finds nothing keeps the last result; a corrupt word (unused bits 26–31 set, or a field over range) reads as all 0 (`E_PAR`). The VC copies it into the IF-05 obstacle fields | **proposed** (#40); needs Buddy 5's and Buddy 2's OK | #40 |

While S5 has a HOLD or MOVE request, the VC's `avoid_step()` in
`robot_control.c` pauses line following and executes S5's moves through the
shared executor `subsystem5/s5_executor.c` (the IF-10/IF-11 hand-shake).

### Event bits (#50)

**Agreed by the team, 21 Nov 2026.** Checked against the code on `main`
(`882f522`). Change it only by team agreement. Every bit or event code that one subsystem raises for
another is listed here: who sets it, who clears it, whether it latches, and
which one wins when several fire at once. The rule: **one producer sets a
bit. Only the producer or the one named consumer clears it.**

There is no shared robot-wide event group. Each signal belongs to its
interface's owner. INT-14 (`int/event_bits`, `testing/integration/test_event_bits.py`)
reads the sources and fails if any file outside the "Producer" and
"Consumer" columns sets, clears, waits on, sends to or reads one of these
objects. Add a writer here first, then in the code and the check.

| Bit / code | Object | Producer (only writer) | Consumer | Set when | Cleared by, and when | Latch? | Priority when several fire |
| --- | --- | --- | --- | --- | --- | --- | --- |
| STOP, bit 0 (`1u`) | `flg_motion_stop` (S2) | `subsystem2_send_command(MOTION_CMD_STOP)` (`subsystem2.c`), for any caller | `motion_task` (`task_motion.c`) | STOP requested | consumer, on read (`TWF_CLR`); nobody calls `tk_clr_flg` | no: consumed once | beats every queued move: the queue is drained in the same cycle |
| `S5_EVT_FOLLOWING` | `flg_s5_evt` (S5) | VC, through `s5_exec_set_following()` (`s5_executor.c`) | S5 (`s5_detect_allowed()`, polled with `TMO_POL`) | VC armed and line-following | VC only (the producer), on any stop, on PAUSE, or when S5 takes control. S5 never clears it: its clears keep every bit except its own reply mask, and it never waits with `TWF_CLR` | level, not latched | — (S5 counts obstacles only while it is set) |
| `S5_EVT_STOPPED` | `flg_s5_evt` | VC executor (`exec_finish()`) | S5 (`task_obstacle.c`) | HOLD done, robot at rest | S5 clears the reply bits it is about to wait for *before* each new request (`s5_post_and_wait()`, `s5_special_move()` for spins and reverses, `forward_watched()` for legs; the watched pass went 23 Nov, #31) | yes, until S5's next request | one reply per request (`b_replied`) |
| `S5_EVT_MOTION_DONE` | `flg_s5_evt` | VC executor | S5 | MOVE finished | as above | yes, until S5's next request | one reply per request |
| `S5_EVT_LINE_FOUND` | `flg_s5_evt` | VC executor | S5 | stop-on-line MOVE saw the line | as above | yes, until S5's next request | one reply per request |
| `S5_EVT_MOVE_FAILED` | `flg_s5_evt` | VC executor | S5 | MOVE rejected, or not finished within 20 s (`S2_AVOID_MOVE_TIMEOUT_MS`). If the VC never replies (it halted), S5 gives up by itself after 25 s (`MOVE_TIMEOUT_MS`) | as above | yes, until S5's next request | S5 treats FAILED as overriding any other bit it sees |
| line event, byte 1 bits 5:3 (`LINE_BARCODE_EVT_*`) | `g_h_line_nav_q` (S3) | `line_service_task` (`line_service.c`, the only `tk_snd_mbf`) | VC only, through `line_service_receive()`. In the test images, the harness that replaces the VC (`all_test`, `line_follow_test`, `line_demo`) | LOST, FOUND, JUNCTION, COMMAND, INVALID, BARCODE, FAULT (STATUS is never queued) | message buffer: each event is read once. `line_service_enable()` empties it, so events from before a stop or pause are dropped | no: a queued event | VC drains up to 24 per cycle and a FAULT ends the drain. **Straddle model** (default, `S2_LINE_STRADDLE=1`, #8): FAULT → line stop; BARCODE → letter kept for the next junction; INVALID logged; LOST, FOUND, JUNCTION, COMMAND ignored (the VC finds junctions itself: C and R both black). **S3 model**: LOST or FAULT → line stop; COMMAND → turn (STRAIGHT carries on); JUNCTION with no mapped barcode → straight on (#72, 21 Nov) |
| line state, byte 1 bits 2:0 (`FOLLOW`…`FAULT`) | S3 snapshot (`line_service_snapshot()`) | `line_service_task` | VC | every sample (1 ms) | overwritten every sample | level | FAULT stops at once. Straddle: PAUSED or a failed snapshot must last 200 ms (`S2_LINE_UNSAFE_MS`); SEARCH (both white) is the normal on-line reading. S3 model: SEARCH, PAUSED or a failed snapshot must last 200 ms |
| line faults, byte 2 bits 7:5 (`LINE_BARCODE_ERR_LINE`, `_QUEUE`, `_TIMING`) | S3 snapshot | `line_service_task` | VC (as state FAULT) | `ERR_LINE`: sensor read failed, or the line was lost longer than `LINE_BARCODE_SEARCH_MS` (2 s; **0 = off** in the `all_test` and `integrated` builds, #8). `ERR_QUEUE`: `g_h_line_nav_q` full. `ERR_TIMING`: a sampling gap | S3 only, in `line_service_calibrate()` or `line_service_enable()`. The VC calls `enable(false)` on every stop and `enable(true)` when it re-arms, so a fault lasts until then | **yes** | any one sets state FAULT |
| `IMU_DATA.motion_state` | S4 snapshot (`mtx_imu_data`) | `imu_task` | VC (stops on LIFTED and FAULT; TILT is report-only, #67), S5 (ignores readings on HUMP and TILT) | every sample (20 ms) | overwritten every sample | level | S4 picks one class per sample: LIFTED > hump just completed > TILT > hump in progress > NORMAL; FAULT on a read failure |
| `IMU_DATA.event` | S4 snapshot | `imu_task` | VC (stops on the LIFT and FAULT events) | for one sample, when it happens. TILT once per tilt episode (`tilt_reported`), LIFT once per lift (`lift_reported`) | next sample | no: one sample wide | as `motion_state`. The VC runs every 10 ms, so it sees every 20 ms sample |
| IMU health (not a bit) | VC, from the S4 snapshot | VC | VC | not calibrated, sensor not OK, a new read error, or no sample for 100 ms (`S2_IMU_STALE_MS`) | — | the stop latches (below) | treated like LIFTED / FAULT: safety stop |
| telemetry faults, bits 0–4 (`S1_FAULT_*`) | IF-05 packet | VC (`robot_control.c`, #72): `S1_FAULT_IMU` when the IMU is unhealthy, `S1_FAULT_MOTOR_STALL` on an encoder stall (only in moves and pivots: line following has no stall fault, #8). In the `all_test` image, `all_buddies_test.c` sets `S1_FAULT_IMU` | PC over MQTT | — | rebuilt from the VC's snapshot each packet (S1 README) | level: the VC's own stop latch holds the memory | independent flags |
| VC stop latch (`halted`, `line_halt`) | `robot_control.c` (internal) | VC | VC | any `halt()` or `halt_line()` | line stops (S3 FAULT, LOST in the S3 model, a stale snapshot): back on the line with the wheels still for 1 s (`S2_LINE_REARM_MS`), and not paused. Every other stop: reboot | **yes** | safety stop (motion fault, IMU, bad S5 word) > PAUSE > S5 avoidance > line following. PAUSE is ignored during a safety stop |
| VC pause latch (`paused`) | `robot_control.c` (internal) | VC, on an MQTT PAUSE (IF-06) | VC | PAUSE while armed | VC, on an MQTT RESUME, only when on the line with a healthy IMU; a paused line stop then restarts its 1 s re-arm wait | **yes** | below a safety stop; blocks the line re-arm |

Notes:

- **Lifting the robot stops it in the VC** (state LIFTED and event LIFT,
  code 3; #53, 21 Nov). It replaced COLLISION, removed on the instructor's
  instructions (13 Nov). TILT no longer stops the robot (#67, decided 20 Nov).
- **`IMU_DATA.event` is one sample wide.** A consumer that polls slower than
  20 ms can miss it. Such a consumer must read `motion_state` or the
  highest-hump field instead. S5 reads `motion_state` only.
- INT-14 checks *who* writes each object. *When* and *in what order* the
  bits fire is covered by INT-03 (VC, S3 and S4 rows, host) and, for the S5
  rows, by INT-13 (to do: INT-03 stubs the S5 executor as idle) and INT-10
  on the robot.

### Other open points

Decided 20 Nov ([DECISIONS.md](DECISIONS.md)): add a START command and a
Ready state (#48, built 21 Nov; bench test pending); tilt is report-only (#67); on line
loss the robot stops and waits (#68).

Still to define (TBD): command handling when Wi-Fi drops (#49).

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The counts are for the final firmware: the objects the code
creates today (the template's producer/consumer code was removed in #74), plus the
**planned** objects marked (+).

| Object | Limit | Kernel | S1 | S2 | S3 | S4 | S5 | Total |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Task | 32 | 3 | 0 | 1 | 1 | 1 | 1 | 7 |
| Semaphore | 16 | 3 | | | | | | 3 |
| Event flag | 16 | 5 | | 1 | | | 1 | 7 |
| Mailbox | 12 | | | | | | | 0 |
| Mutex | 12 | | | 1 | 1 | 1 | | 3 |
| Message buffer | 12 | | 3 + 1 (+) | 1 | 1 | | | 6 |
| Fixed memory pool | 12 | | | | | | | 0 |
| Variable memory pool | 12 | | | | | | | 0 |
| Cyclic handler | 12 | | | | 1 | | | 1 |
| Alarm handler | 12 | | | | | | | 0 |

What each column is:

- **Kernel:** tasks are the initial task, the USB console and `cyw43`.
  Semaphores are the device-driver locks for the ADC, I2C0 and serial
  drivers. Event flags are the device manager lock, the ADC, I2C0 and two for
  UART0.
- **S1:** `telemetry_queue`, `command_queue`, `mapping_queue`
  (`subsystem1.c`); (+) the barcode event queue for IF-13.
- **S2:** `motion_task`; `mbf_motion_cmd`, `mtx_motion_status`,
  `flg_motion_stop` (`subsystem2.c`).
- **S3:** `line_service_task`; `g_h_line_mutex`, `g_h_line_nav_q`, the 1 ms
  `g_h_line_cycle` (`line_service.c`).
- **S4:** `imu_task`, `mtx_imu_data` (`subsystem4.c`).
- **S5:** `obstacle_task`, `flg_s5_evt` (`subsystem5.c`).

Add a row entry here before creating any new object.
