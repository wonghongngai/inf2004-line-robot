# INF2004 Group 17: autonomous line-following robot

An autonomous robot car on a Raspberry Pi Pico W + Robo Pico, running
micro T-Kernel 3.0 on **both cores** of the RP2040, written in C. It follows a
line, decodes Code 39 barcodes, measures humps with an IMU, scans and avoids
obstacles with an ultrasonic sensor, and reports telemetry over Wi-Fi/MQTT.

| Page | What it tells you |
| --- | --- |
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
| --- | --- | --- |
| Purpose | **Try things out**: check a sensor, tune a number, watch what the robot does | **The robot**: what you demo, and where every requirement is proven |
| How it runs | One test loop (`app_program/subsystem2/all_buddies_test.c`) drives the motors itself, with lots of `[ALL TEST]` console output. Safety stops can be made report-only (`ALL_TEST_CONTINUE_ON_SENSOR=1`) | Each buddy's task runs normally; the Vehicle Controller (`robot_control.c`, inside `motion_task`) coordinates them. Safety stops are always on |
| Counts as evidence? | **No** | **Yes**: L4 tests and TEST_LOG results come from this build |

**Decided 13 Nov 2026 (issue #4):** `integrated` is the final firmware.
`all_test` and the other modes are test harnesses only.

> ⚠️ Until issue #8 is done, `integrated` does **not** have the turning,
> straight-driving and avoidance tuning added to `all_test` in early November,
> so expect it to drive worse than `all_test` for now.

### 1.1 Build

Everyone builds with the same script, `build.sh` in the repo root. Run it from Git
Bash, WSL, macOS or Linux, with `arm-none-eabi-gcc` on `PATH` and
`PICO_SDK_PATH` set (section 4). It works from any folder, prints the `.uf2`
it produced, and saves the full output to `build_make/build_<mode>.log`
(with any extra options added to the name, e.g. `build_motion_TEST=1.log`).

**`integrated`**, with Wi-Fi and MQTT (the demo build):

```sh
sh build.sh integrated
```

→ `build_make/robot-integrated.uf2`

**`integrated` with the on-Pico self-tests** (the `TEST_CHECK`s in each
`subsystemN_test.c`): `sh build.sh integrated TEST=1`. About 10 s
after boot it prints one `[TEST] <ID> PASS|FAIL` line per check. This image
does not drive the robot by itself (section 2.1).

**`all_test`**, with Wi-Fi and MQTT: `sh build.sh all_test`
(→ `build_make/robot-all_test.uf2`).
Without Wi-Fi: `sh build.sh all_test_offline`
(→ `build_make/robot-all_test-offline.uf2`).

Every mode builds with `TEST=0 CONSOLE=usb_cdc CRASH_REPORT=0`
(`CRASH_REPORT=0` is required: GP0 is an IR input). Any `VAR=value` after the
mode is passed to make and overrides the preset, e.g.
`sh build.sh all_test ALL_TEST_CONTINUE_ON_SENSOR=1`.
`sh build.sh --help` lists the modes, and `sh build.sh all` builds every one
(section 2.2).

The script runs plain make, so the equivalent by hand is, for example,
`make -C build_make -j8 ROBOT_MODE=integrated WIFI=cyw43 NET=lwip MQTT=1 TEST=0 CONSOLE=usb_cdc CRASH_REPORT=0`.

Wi-Fi builds need `config/wifi_secrets.h` (gitignored, never commit it):
copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h` and fill in
your network (`WIFI_SSID`, `WIFI_PASSWORD`) and broker (`MQTT_BROKER_IP`,
`MQTT_BROKER_PORT`, `MQTT_CLIENT_ID`, `MQTT_TOPIC`).

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

## 2. All firmware modes

Build any of them with `sh build.sh <mode>` (section 1.1). Only
`integrated` and `all_test` use Wi-Fi and MQTT; the others build with
`WIFI=none NET=none MQTT=0`. Every image lands in `build_make/` and is named
`robot-<mode>`, plus a suffix for any setting that differs from the mode's
usual build (section 6). Only `integrated` counts as evidence; every
other mode is a test or bench build.

| Command | Image | What it is for |
| --- | --- | --- |
| `sh build.sh integrated` | `robot-integrated.uf2` | The final robot (section 1) |
| `sh build.sh all_test` | `robot-all_test.uf2` | Whole-robot test harness (section 1) |
| `sh build.sh all_test_offline` | `robot-all_test-offline.uf2` | `all_test` without Wi-Fi (`ROBOT_MODE=all_test ALL_TEST_OFFLINE=1`) |
| `sh build.sh integrated_motor_test` | `robot-integrated_motor_test.uf2` | Wheels raised: S2 motor pulses with S3 IR and S4 IMU sampling, prints PWM/sensor diagnostics |
| `sh build.sh motor_test` | `robot-motor_test.uf2` | Wheels raised: S2 motors only (see `app_program/subsystem2/HARDWARE_TEST.md`) |
| `sh build.sh motion` | `robot-motion.uf2` | S2 motion controller on its own |
| `sh build.sh line_follow_test` | `robot-line_follow_test.uf2` | S2 + S3 open-loop line following, no IMU, encoders or barcodes (`app_program/subsystem2/line_follow_test.c`) |
| `sh build.sh barcode_test` | `robot-barcode_test.uf2` | S3 barcode bench, motors off (`app_program/subsystem3/line_demo.c`) |

### 2.1 Test variants

Add these after the mode. Each one changes the image name, so a variant can't
be mistaken for the normal build.

| Command | Image | What it adds |
| --- | --- | --- |
| `sh build.sh integrated TEST=1` | `robot-integrated-test.uf2` | On-Pico self-tests (the `TEST_CHECK`s in each `subsystemN_test.c`); about 10 s after boot, one `[TEST] <ID> PASS\|FAIL` line per check |
| `sh build.sh motion TEST=1` | `robot-motion-test.uf2` | The same self-tests in the standalone motion build |
| `sh build.sh motion MOTION_DEMO=1` | `robot-motion-demo.uf2` | Scripted S2 motion demo (`motion` only; cannot be combined with `TEST=1`) |
| `sh build.sh all_test ALL_TEST_CONTINUE_ON_SENSOR=1` | same as `all_test` | Safety stops become report-only (diagnostic bypass; never for evidence). More switches in section 3.4 |

> ⚠️ **A `TEST=1` image does not drive the robot by itself.** It leaves out
> the Vehicle Controller (`robot_control.c`) and refuses outside motion
> commands, so only the tests move the motors. Use it to check each
> subsystem's code on the Pico; for driving, demos and L4 track tests, flash
> the normal `TEST=0` image.

`TEST=1` is rejected for `motor_test`, `integrated_motor_test`,
`all_test`, `all_test_offline`, `line_follow_test` and `barcode_test`: these
drive the motors or need live sensors.

### 2.2 Build everything before a pull request

```sh
sh build.sh all
```

Builds every mode in section 2 and every `TEST=1` / `MOTION_DEMO=1` variant
in section 2.1 (11 builds, about a minute), then prints one
`PASS`/`FAIL`/`SKIP` line per build with its `.uf2` or the log to read. It
exits non-zero if any build failed, so a change that breaks a mode you did
not touch shows up before review. Without `config/wifi_secrets.h` the three
Wi-Fi builds are skipped, not failed. Options after `all` are added to every
build, e.g. `sh build.sh all SMP=0`.

Wiring, calibration and motor checks: [app_program/subsystem2/INTEGRATION.md](app_program/subsystem2/INTEGRATION.md),
[HARDWARE_TEST.md](app_program/subsystem2/HARDWARE_TEST.md) and
[CALIBRATION.md](app_program/subsystem2/CALIBRATION.md).

---

## 3. Robot reference

### 3.1 Sensors and pins

Both builds use the **SIDE** line layout (`ALL_TEST_LINE_LAYOUT`,
`S2_LINE_LAYOUT`):

| Device | Pins | Owner |
| --- | --- | --- |
| IR barcode scanner (offset beside the line) | GP28 | S3 |
| IR centre / right (steering) | GP27 / GP0 | S3 |
| IMU LSM303DLHC, I2C0 | GP4 SDA, GP5 SCL | S4 |
| Ultrasonic HC-SR04P | GP16 TRIG, GP17 ECHO | S5 |
| Scanning servo SG90 | GP14 (Robo Pico servo header) | S5 |
| Left motor, Robo Pico M1 | GP8 forward, GP9 reverse | S2 |
| Right motor, Robo Pico M2 | GP10 forward, GP11 reverse | S2 |
| Left encoder | GP26 A, GP6 B | S2 |
| Right encoder | GP2 A, GP3 B | S2 |

IR inputs read raw 1 on black, 0 on white. Never use GP23/GP24/GP25/GP29:
they are wired to the Pico W radio. Claim every pin in `TEAM.md` before wiring it.

### 3.2 MQTT

Topics are prefixed with `MQTT_CLIENT_ID` from `config/wifi_secrets.h`:

| Topic | Direction | Payload |
| --- | --- | --- |
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
config is in `tools/mosquitto-local.conf`.

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
| --- | --- | --- |
| `ALL_TEST_CONTINUE_ON_SENSOR` | `0` | `1` makes sensor detections and faults report-only (diagnostic bypass; never for evidence) |
| `ALL_TEST_DURATION_MS` | `0` | `0` runs continuously; a positive value stops the run after that time |
| `ALL_TEST_LINE_LAYOUT` | `LINE_BARCODE_LAYOUT_SIDE` | Line sensor layout |
| `ALL_TEST_BARCODE_TURNS` | `1` | A stored barcode letter turns the next cross-junction into the mapped move |
| `ALL_TEST_STOP_ON_COLLISION` | `1` | IMU collision stop; collision detection is being removed (issue #53) |

---

## 4. Set up the Pico C SDK and the compiler

You need two separate things:

| What | Why | Where it comes from |
| --- | --- | --- |
| **Pico C SDK** | USB and Wi-Fi driver source code | `git clone` (step 4.2) |
| **Arm GNU Toolchain** (`arm-none-eabi-gcc`) | the compiler | a separate download (step 4.3) |

> ⚠️ **The SDK does NOT include the compiler.** Install the compiler on its
> own and put its `bin` folder on your `PATH` (step 4.4).

### 4.1 Install the basic tools: git, make and a host C/C++ compiler

```sh
# macOS
xcode-select --install

# Linux / WSL (Ubuntu)
sudo apt update && sudo apt install -y git make g++
```

On Windows without WSL, MSYS2 (UCRT64) provides `make`, `gcc` and `g++`.
The host C++ compiler builds `build_make/tools/elf2uf2.exe` on the first
build; the host C compiler runs the host tests.

### 4.2 Download the Pico C SDK (any folder works; this guide uses `~/pico`)

```sh
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

The `submodule` line is required: it downloads the USB, Wi-Fi and TCP/IP
(lwIP) code. SDK 2.3.0 has also been tested and works.

## 4.3 Install the Arm GNU Toolchain (the compiler)

- Download from <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>.
  Pick the **AArch32 bare-metal target (arm-none-eabi)** package for your
  computer (for WSL, the **Linux x86_64** `.tar.xz`) and extract it, e.g. to
  `~/tools/arm-gnu-toolchain-<version>/`.
- **Linux / WSL shortcut:** `sudo apt install -y gcc-arm-none-eabi` puts it on
  your PATH; then skip the PATH line in 4.4.

The folder that contains `arm-none-eabi-gcc` is the toolchain's **`bin`**
folder, e.g. `~/tools/arm-gnu-toolchain-15.3.rel1/bin`.

### 4.4 Tell your shell where the compiler and SDK are

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

If the build later stops with `arm-none-eabi-gcc not found`, fix `PATH`; with
`PICO_SDK_PATH is not set`, fix `PICO_SDK_PATH`.

If you installed the SDK and compiler with the Raspberry Pi Pico VS Code
extension, they are under `~/.pico-sdk/`: the compiler's `bin` folder is
`~/.pico-sdk/toolchain/<version>/bin` and the SDK is
`~/.pico-sdk/sdk/<version>` (see the warning below if your user folder has a
space).

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
make ROBOT_MODE=motion CONSOLE=uart WIFI=none -j8   # no USB console, radio off (no Pico SDK needed)
make TEST=1 -j8                   # firmware with the subsystem self-tests
make ROBOT_MODE=motion CRASH_REPORT=1 -j8           # also see crash reports, on GP0/GP1 (uses those pins)
```

`CONSOLE=uart` and `CRASH_REPORT=1` put UART0 on GP0/GP1, and GP0 is the
right IR sensor. Use them only with a mode that does not read the IR sensors
(`motion` or `motor_test`). Every other mode refuses to build: `line_service.c`
stops with `#error "IR_RIGHT uses GP0..."`. Unplug the IR sensor from GP0
first, or UART transmit and the sensor output drive the same wire. To debug a
crash, e.g.:

```sh
make ROBOT_MODE=motion SMP=0 TEST=1 CRASH_REPORT=1 -j8
#  -> build_make/robot-motion-wifi-smp0-crash-test.uf2
```

| Option | Values (**default**) | What it does | Image name | C macro your code can test |
| --- | --- | --- | --- | --- |
| `SMP` | `0`, **`1`** | `1` runs the kernel on both RP2040 cores; `0` uses core 0 only | `-smp0` / *(nothing)* | `TK_SUPPORT_SMP` (1 or 0) |
| `CONSOLE` | **`usb_cdc`**, `uart` | where `tm_printf` output goes. `usb_cdc` = USB cable; `uart` = UART0 only (GP0 TX / GP1 RX, needs a USB-serial adapter). **Uses GP0/GP1**, so not with the IR sensors (see above) | *(nothing)* / `-uart` | `TM_CONSOLE_USB_CDC` |
| `WIFI` | **`cyw43`**, `none` | `cyw43` powers up the Pico W radio and its driver task; `none` leaves the radio off | network suffix, see below | `TM_WIFI_CYW43` |
| `NET` | **`none`**, `lwip` | `lwip` adds the TCP/IP stack (needs `WIFI=cyw43` and the SDK's `lib/lwip`) | network suffix, see below | `CYW43_LWIP` |
| `MQTT` | **`0`**, `1` | `1` adds the MQTT client (needs `NET=lwip` and `config/wifi_secrets.h`) | network suffix, see below | `TM_MQTT` |
| `TEST` | **`0`**, `1` | `1` compiles every `subsystemN_test.c` and runs the tests at power-up | `-test` / *(nothing)* | `APP_TEST` (1 or 0) |
| `CRASH_REPORT` | **`0`**, `1` | `1` also copies the console to UART0 on **GP0 (TX) / GP1 (RX)**, 115200 baud, so you can see the last messages and the kernel's crash report. **Uses GP0/GP1.** `CONSOLE=uart` always does this | `-crash` / *(nothing)* (`-uart` covers it with `CONSOLE=uart`) | `TM_CONSOLE_UART` (1 or 0) |
| `ROBOT_MODE` | **`integrated`**, see section 2 | which firmware to build | `robot-<mode>` | `APP_ROBOT_INTEGRATED`, `APP_ALL_TEST`, … |

**Image names** are `robot-<mode>[-<network>][-demo][-smp0][-uart|-crash][-test].uf2`.
A suffix appears only when a setting differs from the mode's usual build, so
two different builds never share a name (an SMP=1 image was once flashed in
place of an SMP=0 one). The network suffix names the `WIFI`/`NET`/`MQTT`
combination: `-offline` (`none`/`none`/`0`), `-wifi` (`cyw43`/`none`/`0`),
`-lwip` (`cyw43`/`lwip`/`0`) or `-mqtt` (`cyw43`/`lwip`/`1`). It is left out
when it is the mode's usual one: `mqtt` for `integrated` and `all_test`,
`offline` for every other mode. So a plain `make` (`WIFI=cyw43 NET=none`)
gives `robot-integrated-wifi.uf2`, while `sh build.sh integrated` gives
`robot-integrated.uf2`.

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
| --- | --- | --- |
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
