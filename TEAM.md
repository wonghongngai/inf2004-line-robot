# Team and subsystems

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Name | Student ID | GitHub username | Code | Tests |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | Wi-Fi, MQTT, telemetry and commands | Kan Ka En | 2503598 | `dunnknow` | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motors, encoders, PID; hosts the Vehicle Controller (`robot_control.c`) | Kang Zhi Xian | 2503986 | `2andbeii` | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | IR line following, Code 39 barcode, junctions | Chai Sook Hui | 2502871 | `chaisookhui` | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU (LSM303DLHC): humps, tilt, lift-stop, collision | Lew Zhi Xin | 2503689 | `lewzhixin` | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Ultrasonic + servo scanning, obstacle profiling, avoidance | Wong Hong Ngai | 2500434 | `wonghongngai` | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin.

| Pin | Owner | Used for |
| --- | --- | --- |
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S3 | right IR digital input; USB console only, CRASH_REPORT=0 |
| GP1 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP2 | S2 | right encoder A |
| GP3 | S2 | right encoder B |
| GP4 | S4 | I2C0 SDA, LSM303DLHC |
| GP5 | S4 | I2C0 SCL, LSM303DLHC |
| GP6 | S2 | left encoder B |
| GP7 | S3 | left IR digital input |
| GP8 | S2 | left motor forward PWM |
| GP9 | S2 | left motor reverse (zero PWM) |
| GP10 | S2 | right motor forward PWM |
| GP11 | S2 | right motor reverse (zero PWM) |
| GP12 | | |
| GP13 | | |
| GP14 | S5 | SG90 servo PWM (slice 7A, Robo Pico servo port 4); was the kernel's unused I2C1 SDA boot pin, S5 takes it over with `pwm_set_pin()` |
| GP15 | kernel I2C1 | unused SCL boot assignment |
| GP16 | S5 | HC-SR04P TRIG (SIO output, Grove 4) |
| GP17 | S5 | HC-SR04P ECHO (PWM slice 0B input, gated echo timing, Grove 4) |
| GP18 | | |
| GP19 | | |
| GP20 | | |
| GP21 | | |
| GP22 | | |
| GP26 | S2 | left encoder A; ADC pin initialization disabled |
| GP27 | S3 | centre IR digital input; ADC pin initialization disabled |
| GP28 | | (ADC2 capable) |

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
| UART1 | | | |
| I2C0 | S4 via kernel device `iica` | `imu_task` (plus existing startup probe) | GP4/GP5 |
| I2C1 | kernel, unused by application | kernel driver | GP14/GP15 |
| SPI0 | | | |
| SPI1 | | | |
| ADC | | | GP26–GP28 |
| PWM slices 4, 5 | S2 | `motion_task` or bench task (mutually exclusive) | GP8-GP11 |
| PWM slices 0, 7 | S5 | `obstacle_task` | GP17 (echo timing), GP14 (servo) |
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

- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `app_program/subsystem1/mqtt_manager.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest and 32 the lowest (`CNF_MAX_TSKPRI` in `config/config.h`).
1–4 are used by the kernel. Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

| Subsystem | Priority band |
| --- | --- |
| kernel | 1–4 |
| *(unused)* | 5–11 |
| 1 | 12–13 |
| 2 | 14–15 |
| 3 | 16–17 |
| 4 | 18–19 |
| 5 | 20–21 |

Every task in the firmware, highest priority first. "Name" is the string
passed to `app_start_task()`; the entry function is given where it differs.

| Task | Owner | Priority | Core | Built in `ROBOT_MODE` | Source |
| --- | --- | --- | --- | --- | --- |
| initial task (runs `usermain()`, then the `TEST=1` self-tests, then sleeps) | kernel | 1 | — | all | `kernel/inittask/inittask.c` |
| USB console service | kernel | 3 | core 0 | all (`CONSOLE=usb_cdc`) | `lib/libtm/.../usb/usb_console.c` |
| `cyw43` (`cyw43_utk_task`): Wi-Fi, lwIP, MQTT, and S1's queues | kernel | 4 | core 0 | Wi-Fi builds | `lib/libwifi/.../cyw43_utk_kernel.c` |
| — | S1 | — | — | — | no task of its own: `subsystem1_start()` only checks its queues |
| `motion_task`: PID loop and the Vehicle Controller | S2 | 14 | core 1 | `integrated`, `motion` | `subsystem2/subsystem2.c` |
| `motion_task` (entry `subsystem2_hw_test_task`): motor pulse bench | S2 | 14 | core 1 | `integrated_motor_test`, `motor_test` | `subsystem2/subsystem2.c` |
| *example:* `producer_task` (compatibility) | S2 | 15 | core 1 | `integrated`, `integrated_motor_test` | `subsystem2/task_producer.c` |
| `all_buddies_test` (entry `all_task`): whole-robot test loop | S2 | 15 | core 1 | `all_test` | `subsystem2/all_buddies_test.c` |
| `line_follow_test` (entry `follow_task`) | S2 | 15 | core 1 | `line_follow_test` | `subsystem2/line_follow_test.c` |
| `line_service_task`: IR sampling, barcode, junctions | S3 | 16 | core 1 in `integrated*` and `all_test`; core 0 in `line_follow_test`, `barcode_test` | all except `motion`, `motor_test` | `subsystem3/line_service.c` |
| *example:* `consumer_task` (compatibility) | S3 | 17 | core 0 | `integrated`, `integrated_motor_test` | `subsystem3/task_consumer.c` |
| `line_demo_task`: barcode bench | S3 | 17 | core 0 | `barcode_test` | `subsystem3/line_demo.c` |
| `imu_task` | S4 | 18 | any | `integrated*`, `all_test` | `subsystem4/subsystem4.c` |
| `obstacle` (entry `obstacle_task`): scan, profile, avoid | S5 | 20 | any | `integrated*`, `all_test` | `subsystem5/subsystem5.c` |

`integrated*` means `integrated` and `integrated_motor_test`. The test loops
at 15 replace `motion_task`: they drive the motors themselves, so the two are
never built together.

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
| --- | --- | --- | --- | --- | --- |
| IF-01 | S2 → S3 | mailbox (owned by S3) | `mbx_samples` | `SAMPLE_MSG` in `subsystem2.h` | *example* |
| IF-02 | S3 → S2 | semaphore + fixed memory pool (owned by S2) | `sem_slots`, `mpf_samples` | receiver returns the block, then 1 slot | *example* |
| IF-03 | S3 → S4 | message buffer + event flag (owned by S4) | `mbf_summary`, `flg_events` | text line ≤ `S4_SUMMARY_MAX` bytes, then `FLG_BATCH` | *example* |
| IF-04 | S4 → S3 | shared data under a mutex (owned by S3) | `app_stats`, `mtx_stats` | `APP_STATS`, read only while locked | *example* |
| IF-05 | All → S1 | telemetry queue | `subsystem1_publish_telemetry` | nonblocking message buffer | Buddy 1 |
| IF-06 | S1 → controller | command queue | `subsystem1_receive_command` | message buffer receive | Buddy 1 |
| IF-07 | S3 to S2 robot adapter | S3 message buffer and snapshot API | `g_h_line_nav_q` | line positions and barcode/junction commands; one consumer, demo disabled | integrated build |
| IF-08 | S4 to S2 robot adapter | snapshot under S4 mutex | `subsystem4_get_latest()` | IMU readiness, terrain and safety stop | integrated build |
| IF-09 | Application to S2 | message buffer, separate STOP flag | `mbf_motion_cmd`, `flg_motion_stop` | `MOTION_COMMAND`; autonomous profiles reject external moves | motion build |
| IF-10 | S5 → S2 robot adapter (VC), and the `all_test` loop | packed 32-bit request word, single writer, read with `subsystem5_get_request()` (owned by S5) | `subsystem5_get_request()` | `S5_REQUEST`: NONE / HOLD / MOVE{fwd cm, pivot deg, stop-on-line (first sensor, or with bit 28 `line_all` once every steering sensor has crossed)} / RESUME / SAFE_STOP plus a 4-bit seq; reserved values → VC stops (`E_PAR`). S5 posts HOLD at boot and the VC does not arm until S5 releases it (after the servo's start-up swing) | **proposed** by Buddy 5; needs Buddy 2's OK |
| IF-11 | S2 robot adapter (VC), or the `all_test` loop → S5 | event flag (owned by S5) | `flg_s5_evt` | `S5_EVT_FOLLOWING` / `STOPPED` / `MOTION_DONE` / `LINE_FOUND` / `MOVE_FAILED` | **proposed** by Buddy 5; needs Buddy 2's OK |

Robot modes and validation are documented in
`app_program/subsystem2/INTEGRATION.md`. Active S2 tasks: motion/bench at 14 on
core 1; compatibility producer at 15 only in integrated builds. S3 retains
consumer at 17 on core 0, and runs the line service at 16 on core 1 in
`integrated` and `all_test` (away from the radio on core 0; core 0 in the
other modes). S4 IMU runs at 18 on any core.
S5 `obstacle_task` runs at 20 on any core in `integrated` and
`integrated_motor_test`; S1 is not started by these robot profiles. While S5
has a HOLD or MOVE request, the VC's `avoid_step()` hook in `robot_control.c`
pauses line following and executes S5's moves; in `all_test` its loop
does the same through `s5_step()` in `all_buddies_test.c`. Both run the one
shared executor `subsystem5/s5_executor.c` (the IF-10/IF-11 hand-shake) and
only supply their own drive functions (`S5_EXEC_OPS`). The 1 ms kernel timer supports
the short requested encoder sleep; actual polling includes an extra tick of
kernel rounding and scheduling overhead.

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The counts below are what the code creates in
`ROBOT_MODE=integrated` with Wi-Fi and the USB console (audited 2026-11-14).

| Object | Limit | Kernel | S1 | S2 | S3 | S4 | S5 | Total |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Task | 32 | 3 | 0 | 2 | 2 | 1 | 1 | 9 |
| Semaphore | 16 | 3 | | 1 | | | | 4 |
| Event flag | 16 | 5 | | 1 | | | 1 | 7 |
| Mailbox | 12 | | | | 1 | | | 1 |
| Mutex | 12 | | | 1 | 2 | 1 | | 4 |
| Message buffer | 12 | | 3 | 1 | 1 | | | 5 |
| Fixed memory pool | 12 | | | 1 | | | | 1 |
| Variable memory pool | 12 | | | | | | | 0 |
| Cyclic handler | 12 | | | | 1 | | | 1 |
| Alarm handler | 12 | | | | | | | 0 |

What each column is:

- **Kernel:** tasks are the initial task, the USB console and `cyw43`.
  Semaphores are the device-driver locks for the ADC, I2C0 and serial
  drivers. Event flags are the device manager lock, the ADC, I2C0 and two for
  UART0. A build without Wi-Fi or the USB console has one task fewer for each.
- **S1:** `telemetry_queue`, `command_queue`, `mapping_queue`
  (`subsystem1.c`). No task: `cyw43` services the queues.
- **S2:** `motion_task`, `producer_task`; `mbf_motion_cmd`,
  `mtx_motion_status`, `flg_motion_stop` (`subsystem2.c`); *example:*
  `sem_slots`, `mpf_samples` (`task_producer.c`).
- **S3:** `line_service_task`, `consumer_task`; `g_h_line_mutex`,
  `g_h_line_nav_q`, the 1 ms `g_h_line_cycle` (`line_service.c`); *example:*
  `mbx_samples`, `mtx_stats` (`subsystem3.c`).
- **S4:** `imu_task`, `mtx_imu_data` (`subsystem4.c`).
- **S5:** `obstacle_task`, `flg_s5_evt` (`subsystem5.c`).

Other modes stay within these totals. `producer_task`, `consumer_task` and
the *example* objects exist only in `integrated` and `integrated_motor_test`.
The `all_test` and `line_follow_test` loops replace `motion_task`, `barcode_test`
adds `line_demo_task`, and `motion` and `motor_test` create no S3–S5 objects
(section 4).

Buddy 1 integration: MQTT implementation moved from the library into subsystem1;
the service entry point remains `mqtt_utk_start`. Subsystem4 no longer pauses the
removed LED demo. Team review of this shared build/interface change is required
before merging. See `app_program/subsystem1/README.md`.
