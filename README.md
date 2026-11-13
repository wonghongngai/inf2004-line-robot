# INF2004 Group 17: autonomous line-following robot

An autonomous robot car on a Raspberry Pi Pico W + Robo Pico, running
micro T-Kernel 3.0 on **both cores** of the RP2040, written in C. It follows a
line, decodes Code 39 barcodes, measures humps with an IMU, scans and avoids
obstacles with an ultrasonic sensor, and reports telemetry over Wi-Fi/MQTT.

| Page | What it tells you |
|---|---|
| [STATUS.md](STATUS.md) | What is done and what is left, with evidence |
| [testing/TEST_PLAN.md](testing/TEST_PLAN.md) | What each buddy tests next, and how |
| [testing/README.md](testing/README.md) | How to run and record a test |
| [DEVIATIONS.md](DEVIATIONS.md) | Where the code differs from the Week 6 / PreTri 1 plan, and why |
| [TEAM.md](TEAM.md) | Owners, pins and interfaces |
| [GitHub issues](https://github.com/wonghongngai/inf2004-line-robot/issues) | Tasks, prefixed `S1:`–`S5:`, `INT:`, `SYS:`, `Team:`, `Repo:` |

New to the project? Set up the SDK and compiler first (section 4), then come
back to section 1.

---

## 1. Which firmware to use: `integrated` or `all_test`

The same code builds into several firmware images, chosen with
`ROBOT_MODE`. Two matter day to day:

| | `ROBOT_MODE=all_test` | `ROBOT_MODE=integrated` |
|---|---|---|
| Purpose | **Try things out**: check a sensor, tune a number, watch what the robot does | **The robot**: what you demo, and where every requirement is proven |
| How it runs | One test loop (`app_program/subsystem2/all_buddies_test.c`) drives the motors itself, with lots of `[ALL TEST]` console output. Safety stops can be made report-only (`ALL_TEST_CONTINUE_ON_SENSOR=1`) | Each buddy's task runs normally; the Vehicle Controller (`robot_control.c`, inside `motion_task`) coordinates them. Safety stops are always on |
| Counts as evidence? | **No** | **Yes**: L4 tests and TEST_LOG results come from this build |

**Decided 13 Nov 2026 (issue #4):** `integrated` is the final firmware.
`all_test` and the other modes are test harnesses only.

> ⚠️ Until issue #8 is done, `integrated` does **not** have the turning,
> straight-driving and avoidance tuning added to `all_test` in early November,
> so expect it to drive worse than `all_test` for now.

### 1.1 Build

Run from the repo root (Git Bash, WSL, macOS or Linux) with `PICO_SDK_PATH`
set (section 4). `CRASH_REPORT=0` is required: GP0 is an IR input.

**`integrated`**, with Wi-Fi and MQTT (the demo build):

```sh
make -C build_make -j8 ROBOT_MODE=integrated WIFI=cyw43 NET=lwip MQTT=1 TEST=0 CONSOLE=usb_cdc CRASH_REPORT=0
```

→ `build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_integrated.uf2`

**`integrated` with the on-Pico self-tests** (the `TEST_CHECK`s in each
`subsystemN_test.c`): the same command with `TEST=1`. About 10 s after boot it
prints one `[TEST] <ID> PASS|FAIL` line per check.

**`all_test`**, with Wi-Fi and MQTT:

```sh
make -C build_make -j8 ROBOT_MODE=all_test NET=lwip MQTT=1 TEST=0 CONSOLE=usb_cdc CRASH_REPORT=0
```

→ `build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_all_test.uf2`

To run `all_test` without Wi-Fi, replace `NET=lwip MQTT=1` with
`ALL_TEST_OFFLINE=1 WIFI=none NET=none MQTT=0`
(→ `build_make/mtk3pico_smp1_usb_cdc_all_test.uf2`).

Wi-Fi builds need `config/wifi_secrets.h` (gitignored, never commit it). It
defines your network and broker: `WIFI_SSID`, `WIFI_PASSWORD`,
`MQTT_BROKER_IP`, `MQTT_CLIENT_ID` and related settings (see
`lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c` and
`app_program/subsystem1/mqtt_manager.c`). The `wifi_secrets.example.h` that
the makefile mentions is not in the repo yet; ask a teammate for theirs.

The mode and options are part of the image name, so check you are flashing
the one you meant.

### 1.2 Flash and watch

1. Hold **BOOTSEL**, plug in the USB cable, release. A drive called **RPI-RP2** appears.
2. Copy the `.uf2` onto it. The Pico reboots by itself.
3. Open the USB serial port at 115200 baud. On Windows,
   `testing/subsystem5/serial_monitor.ps1` reconnects after resets and saves
   the log to `testing/subsystem5/logs/` (raw logs are gitignored; commit a
   trimmed copy as test evidence). PuTTY, `screen` or the VS Code Serial
   Monitor also work.

Keep the robot still for the first second after boot: the IMU calibrates its
baseline then. Both builds wait 5 s before driving.

### 1.3 The workflow for any change

1. **Try it in `all_test`**, wheels raised or on the track, using the extra
   console output, until you find the right numbers or behaviour.
2. **Put the change in the shared code**: the integrated path
   (`robot_control.c`, `motion_control.c`) or your own subsystem's files. Never
   leave a fix only in `all_buddies_test.c`.
3. **Run the host tests**: `sh testing/run_host_tests.sh` (add a name filter
   such as `s3/` or `int/` for a subset).
4. **Flash `integrated` and test it there.** Save the log and add a Results
   row to the right `testing/**/TEST_LOG.md` with the commit hash.
5. **Only then** update the cell in [STATUS.md](STATUS.md).

---

## 2. Other firmware modes

| `ROBOT_MODE=` | What it is for |
|---|---|
| `integrated` | The final robot (section 1) |
| `all_test` | Whole-robot test harness (section 1) |
| `integrated_motor_test` | Wheels raised: S2 motor pulses with S3 IR and S4 IMU sampling, prints PWM/sensor diagnostics |
| `motor_test` | Wheels raised: S2 motors only (see `app_program/subsystem2/HARDWARE_TEST.md`) |
| `motion` | S2 motion controller on its own (`MOTION_DEMO=1` for a scripted demo) |
| `line_follow_test` | S2 + S3 open-loop line following, no IMU, encoders or barcodes (`app_program/subsystem2/line_follow_test.c`) |
| `barcode_test` | S3 barcode bench, motors off (`app_program/subsystem3/line_demo.c`) |

Wiring, calibration and motor checks: [app_program/subsystem2/INTEGRATION.md](app_program/subsystem2/INTEGRATION.md),
[HARDWARE_TEST.md](app_program/subsystem2/HARDWARE_TEST.md) and
[CALIBRATION.md](app_program/subsystem2/CALIBRATION.md).

---

## 3. Robot reference

### 3.1 Sensors and pins

Both builds use the **SIDE** line layout (`ALL_TEST_LINE_LAYOUT`,
`S2_LINE_LAYOUT`):

| Device | Pins | Owner |
|---|---|---|
| IR barcode scanner (offset beside the line) | GP28 | S3 |
| IR centre / right (steering) | GP27 / GP0 | S3 |
| IMU LSM303DLHC, I2C0 | GP4 SDA, GP5 SCL | S4 |
| Ultrasonic HC-SR04P | GP16 TRIG, GP17 ECHO | S5 |
| Scanning servo SG90 | GP14 (Robo Pico servo header) | S5 |
| Motors / encoders | Robo Pico M1 (left), M2 (right); see `INTEGRATION.md` | S2 |

IR inputs read raw 1 on black, 0 on white. Never use GP23/GP24/GP25/GP29:
they are wired to the Pico W radio. Claim every pin in `TEAM.md` before wiring it.

### 3.2 MQTT

Topics are prefixed with `MQTT_CLIENT_ID` from `config/wifi_secrets.h`:

| Topic | Direction | Payload |
|---|---|---|
| `<id>/telemetry` | car → PC | 10-byte binary packet once a second (layout in `app_program/subsystem1/README.md`) |
| `<id>/command` | PC → car | 1 raw byte: STATUS, PAUSE, RESUME (RESET_STATS is rejected) |
| `<id>/mapping` | PC → car | 3-byte letter-to-movement map (left, right, straight, U-turn) |

With Mosquitto clients installed, replace `BROKER` and `CLIENT_ID`:

```sh
mosquitto_sub -h BROKER -t 'CLIENT_ID/telemetry' -F '%x' | python3 tools/mqtt_binary.py stream
python3 tools/mqtt_binary.py command pause | mosquitto_pub -h BROKER -t 'CLIENT_ID/command' -s
python3 tools/mqtt_binary.py command resume | mosquitto_pub -h BROKER -t 'CLIENT_ID/command' -s
```

Commands are raw bytes, never text or retained messages. A local broker
config is in `mosquitto-local.conf`.

**Barcode map:** the instructor sends the four letters just before the demo,
so the map must come over MQTT. Today both builds still use a hard-coded map
and `tools/mqtt_binary.py` cannot send a map yet; both are tracked in issue #38.
Until a map arrives, the robot goes straight at junctions.

**Telemetry fields not filled yet:** battery, and obstacle distance, scan
angle and width (issue #40). They read as zero placeholders, not measurements.

### 3.3 Host tests

```sh
sh testing/run_host_tests.sh          # every PC-side test, with a PASS/FAIL summary
sh testing/run_host_tests.sh s3/      # one area
```

Run them before every pull request. See [testing/README.md](testing/README.md).

### 3.4 `all_test` switches

Set in `app_program/subsystem2/all_buddies_test.h` (or with `-D` on the make line):

| Switch | Default | Effect |
|---|---|---|
| `ALL_TEST_CONTINUE_ON_SENSOR` | `0` | `1` makes sensor detections and faults report-only (diagnostic bypass; never for evidence) |
| `ALL_TEST_DURATION_MS` | `0` | `0` runs continuously; a positive value stops the run after that time |
| `ALL_TEST_LINE_LAYOUT` | `LINE_BARCODE_LAYOUT_SIDE` | Line sensor layout |
| `ALL_TEST_BARCODE_TURNS` | `1` | A stored barcode letter turns the next cross-junction into the mapped move |
| `ALL_TEST_STOP_ON_COLLISION` | `1` | IMU collision stop; collision detection is being removed (issue #53) |

---

## 4. Set up the Pico C SDK and the compiler

You need two separate things:

| What | Why | Where it comes from |
|---|---|---|
| **Pico C SDK** | USB and Wi-Fi driver source code | `git clone` (step 4.2) |
| **Arm GNU Toolchain** (`arm-none-eabi-gcc`) | the compiler | a separate download (step 4.3) |

> ⚠️ **The SDK does NOT include the compiler.** Install the compiler on its
> own and put its `bin` folder on your `PATH` (step 4.4).

**4.1 Install the basic tools: git, make and a host C/C++ compiler**

```sh
# macOS
xcode-select --install

# Linux / WSL (Ubuntu)
sudo apt update && sudo apt install -y git make g++
```

On Windows without WSL, MSYS2 (UCRT64) provides `make`, `gcc` and `g++`.
The host C++ compiler builds `build_make/tools/elf2uf2.exe` on the first
build; the host C compiler runs the host tests.

**4.2 Download the Pico C SDK** (any folder works; this guide uses `~/pico`)

```sh
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

The `submodule` line is required: it downloads the USB, Wi-Fi and TCP/IP
(lwIP) code. SDK 2.3.0 has also been tested and works.

**4.3 Install the Arm GNU Toolchain (the compiler)**

- Download from <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>.
  Pick the **AArch32 bare-metal target (arm-none-eabi)** package for your
  computer (for WSL, the **Linux x86_64** `.tar.xz`) and extract it, e.g. to
  `~/tools/arm-gnu-toolchain-<version>/`.
- **Linux / WSL shortcut:** `sudo apt install -y gcc-arm-none-eabi` puts it on
  your PATH; then skip the PATH line in 4.4.

The folder that contains `arm-none-eabi-gcc` is the toolchain's **`bin`**
folder, e.g. `~/tools/arm-gnu-toolchain-15.3.rel1/bin`.

**4.4 Tell your shell where the compiler and SDK are**

Add these two lines to the end of `~/.zshrc` (macOS) or `~/.bashrc`
(Linux/WSL/Git Bash), **changing both paths to match your computer**:

```sh
export PATH="$HOME/tools/arm-gnu-toolchain-15.3.rel1/bin:$PATH"   # folder that contains arm-none-eabi-gcc
export PICO_SDK_PATH="$HOME/pico/pico-sdk"                         # the SDK from step 4.2
```

Open a **new** terminal, then check both:

```sh
arm-none-eabi-gcc --version     # must print a version, not "command not found"
ls $PICO_SDK_PATH               # must list the SDK files
```

If `make` later stops with `arm-none-eabi-gcc not found`, fix `PATH`; with
`PICO_SDK_PATH is not set`, fix `PICO_SDK_PATH`.

> ⚠️ **`PICO_SDK_PATH` must not contain spaces.** make splits paths on
> spaces, so an SDK under a Windows user folder such as `C:/Users/First Last/`
> fails with "does not look like a Pico C SDK folder". Use the folder's short
> name instead (find it with `cmd /c dir /x C:\Users`), e.g.
> `export PICO_SDK_PATH="C:/Users/FIRSTL~1/.pico-sdk/sdk/2.3.0"`, or move the
> SDK to a path without spaces.

---

## 5. Code layout

- **`app_program/app_main.c` → `usermain()` is where the program starts.** In
  `integrated` it calls each subsystem's `subsystemN_init()` (create kernel
  objects), then `subsystemN_start()` (start tasks). `all_test`,
  `line_follow_test` and `barcode_test` start their own test task instead.
- **One folder per buddy:** `app_program/subsystem1/` (Wi-Fi/MQTT) …
  `subsystem5/` (ultrasonic). `subsystemN.h` is that subsystem's public
  interface; `app.h` holds team-wide definitions.
- **Drivers go in your subsystem folder** as `drv_<name>.c` / `drv_<name>.h`
  (e.g. `subsystem5/drv_ultrasonic.c`). Every `.c` file under `app_program/`
  is compiled automatically. Do not add drivers under `device/`: that holds
  the kernel's own drivers, shared by the whole team.
- **Branches:** `main` is the only long-lived branch. Branch off `main`, open a
  pull request back into it, and delete the branch after merging.

---

## 6. Build options: `SMP`, `CONSOLE`, `WIFI`, `NET`, `MQTT`, `TEST`, `CRASH_REPORT`

Options go after `make` in any order. They are not remembered: give them on
every `make`, as the next plain `make` goes back to the defaults.

```sh
make -j8                          # defaults: SMP=1 CONSOLE=usb_cdc WIFI=cyw43 TEST=0 CRASH_REPORT=0
make SMP=0 -j8                    # single-core build
make CONSOLE=uart WIFI=none -j8   # no USB console, radio off (no Pico SDK needed)
make TEST=1 -j8                   # firmware with the subsystem self-tests
make CRASH_REPORT=1 -j8           # also see crash reports, on GP0/GP1 (uses those pins)
```

To debug a crash with `CRASH_REPORT=1`, use a mode that does not use GP0 as
an IR input, e.g.:

```sh
make ROBOT_MODE=motion SMP=0 TEST=1 CRASH_REPORT=1 -j8
#  -> build_make/mtk3pico_smp0_usb_cdc_wifi_crash_motion_test.uf2
```

| Option | Values (**default**) | What it does | Image name | C macro your code can test |
|---|---|---|---|---|
| `SMP` | `0`, **`1`** | `1` runs the kernel on both RP2040 cores; `0` uses core 0 only | `smp1` / `smp0` | `TK_SUPPORT_SMP` (1 or 0) |
| `CONSOLE` | **`usb_cdc`**, `uart` | where `tm_printf` output goes. `usb_cdc` = USB cable; `uart` = UART0 only (GP0 TX / GP1 RX, needs a USB-serial adapter) | `_usb_cdc` / `_uart` | `TM_CONSOLE_USB_CDC` |
| `WIFI` | **`cyw43`**, `none` | `cyw43` powers up the Pico W radio and its driver task; `none` leaves the radio off | `_wifi` / *(nothing)* | `TM_WIFI_CYW43` |
| `NET` | **`none`**, `lwip` | `lwip` adds the TCP/IP stack (needs `WIFI=cyw43` and the SDK's `lib/lwip`) | `_lwip` / *(nothing)* | `CYW43_LWIP` |
| `MQTT` | **`0`**, `1` | `1` adds the MQTT client (needs `NET=lwip` and `config/wifi_secrets.h`) | `_mqtt` / *(nothing)* | `TM_MQTT` |
| `TEST` | **`0`**, `1` | `1` compiles every `subsystemN_test.c` and runs the tests at power-up | `_test` / *(nothing)* | `APP_TEST` (1 or 0) |
| `CRASH_REPORT` | **`0`**, `1` | `1` also copies the console to UART0 on **GP0 (TX) / GP1 (RX)**, 115200 baud, so you can see the last messages and the kernel's crash report. **Uses GP0/GP1.** `CONSOLE=uart` always does this | `_crash` / *(nothing)* | `TM_CONSOLE_UART` (1 or 0) |
| `ROBOT_MODE` | **`integrated`**, see section 2 | which firmware to build | `_<mode>` | `APP_ROBOT_INTEGRATED`, `APP_ALL_TEST`, … |

**Why `CRASH_REPORT` exists.** USB output waits in a buffer until the
kernel's USB task sends it. If the program crashes, no task runs again, so
the last messages and the crash report never reach your USB terminal. UART
output is sent at once, even inside the crash handler. So when you need to
know why the Pico crashed or froze, build with `CRASH_REPORT=1`, connect a
USB-serial adapter (adapter RX → GP0, adapter GND → GND), and read it at
115200 baud. Unplug the IR sensor on GP0 first.

Switching options needs no `make clean`: the makefile notices and recompiles
everything.

### Where the options live in the makefile

All options are handled in **`build_make/pico_rp2040.mk`**. Each goes through
the same steps (search the file for the names in the middle column):

| Step | What to look for | Example for `TEST` |
|---|---|---|
| 1. Default value | `NAME ?= value` near the top of the file | `TEST ?= 0` |
| 2. Allowed values | an `$(error Unknown ...)` check just below it | `$(error Unknown TEST '$(TEST)'; use 0 or 1)` |
| 3. Image-name suffix | `NAME_SUFFIX :=` and the `EXE_FILE :=` line | `TEST_SUFFIX := _test` |
| 4. C macro for the code | a `CFLAGS += -D...` line | `CFLAGS += -DAPP_TEST=$(TEST)` |
| 5. Rebuild on change | the `PROFILE_ID :=` line | `...-test$(TEST)` |

Adding your own option: copy how `TEST` is done (steps 1 to 5). **Step 5
matters:** without it, switching the option can quietly link files that
were compiled with the old setting. Kernel limits (maximum tasks,
semaphores, …) are not make options; they are in `config/config.h`
(`CNF_MAX_...`).

---

*micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under the
T-License 2.2. This dual-core RP2040 port is derived from the official
μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*
