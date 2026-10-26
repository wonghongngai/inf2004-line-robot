# Subsystem 3: line following and barcode

> **BEFORE version — minimum-bit change requirement NOT applied.**
> Every field uses its own natural C type (`bool` per sensor, `enum` for
> state/event/command/fault/position, `char` letter, `uint16_t`/`uint32_t`
> counters). No bit-fields, masks or packed bytes. Same behaviour and tests
> as the packed (after) version, so the two can be compared for Week 13.

The implementation uses the supplied Week 6 pin table and FR3-1–FR3-3,
Week 8 decisions, Change Requirement 1, and the barcode specification/samples.
Instructions inside tribunal templates are treated as document content; no team
submission entries or physical test evidence have been invented.

`line_service.c` owns the three digital IR inputs in one micro T-Kernel task.
`line_barcode.c` is independent of the RTOS and tested on a computer. It estimates
steering position, detects line loss/recovery, and decodes one Code 39 letter
between star markers. It never accesses motors, encoders, WiFi, or another
buddy's peripherals. Motor/PID execution stays with Buddy 2.

The existing consumer/statistics files remain as a compatibility adapter because
the unchanged S2/S4 examples reference them. Their task runs at priority 17;
the IR task runs at 16. Remove the adapter together with the S2/S4 examples when
the team integrates its real subsystems.

## Confirmed barcode and track facts

The new files were found in the sibling `Project` directory. Their source hashes
and independent sample-width fixtures are in `testing/subsystem3/fixtures/`.

- `Barcode & Line Specification.pdf`: 18 mm guide line, 20 mm gap from line edge
  to barcode edge, approximately 141 mm barcode length, barcode beside the line.
- `Barcode Sample.pdf`: A and Z at two print scales. Vector widths confirm
  Code 39 `*A*` and `*Z*`, 29 alternating black/white runs, about 3:1 wide:narrow.
- `BARCODE.pptx`: letter examples include A, F, Z, W, X and Y. Its raster example
  includes an expired-generator watermark; do not print a watermark across the
  sensor scan path.

## Demo flow (instructor brief)

1. The robot scans a Code 39 barcode in either direction and decodes A-Z.
2. The letter is **stored** (BARCODE event; a newer barcode replaces it).
3. Line following continues until the next **cross-junction**.
4. At the junction the stored letter is looked up: a mapped letter gives a
   COMMAND event (movement + letter), an unmapped one a plain JUNCTION.
   The stored letter is then cleared.
5. Repeat for the next barcode.

The letter-to-movement mapping is **not** fixed in the decoder. It is set at
run time with `line_service_set_mapping(left, right, straight, uturn)`, which
Buddy 1 calls when the MQTT mapping message arrives (allowed at any time; a
bad message leaves the old map). Until then `line_demo.h` loads a temporary
map: A=LEFT, B=RIGHT, C=STRAIGHT, Z=U-TURN.

### Bench demo printout

`line_demo.c` (enabled by `LINE_DEMO_ENABLE` in `line_demo.h`, never in test
builds) calibrates, loads the temporary map, enables the service and prints
every event on the USB console, plus a sensor line every second:

```
[S3] demo map: A=LEFT B=RIGHT C=STRAIGHT Z=U-TURN
[S3] sensors L=0 C=1 R=1 (1=black) pos=centre state=FOLLOW waiting=-
[S3] BARCODE 'A' stored, waiting for junction
[S3] JUNCTION -> action LEFT (letter A)
```

It drives no motors and clears latched faults after 1 s so bench testing can
continue. It is the only reader of `line_nav_q`: set `LINE_DEMO_ENABLE` to 0
when the Vehicle Controller takes over. If L/C/R read inverted on black,
change `LINE_DEMO_BLACK_RAW` from `7u` to `0u`.

Multi-letter strings, numeric payloads and checksums are not supported by
this single-letter sample contract.

The Code 39 element table was cross-checked with the
[ZXing reference reader](https://github.com/zxing/zxing/blob/master/core/src/main/java/com/google/zxing/oned/Code39Reader.java).
The implementation is independently written; the sample width fixtures come
from the supplied PDF rectangles rather than the decoder's own lookup table.

## Wiring, mounting profiles and calibration

| Input | GPIO | SIDE profile (default) | TRACK profile |
|---|---|---|---|
| Left IR D0 | GP7 | offset barcode scanner | left line sensor |
| Centre IR D0 | GP27 | left edge of guide line | centre line sensor |
| Right IR D0 | GP0 | right edge of guide line | right line sensor |

Supply sensors at 3.3 V with common ground. GP28/GP1 analogue outputs are unused.
GP0 conflicts with UART0: use USB console and `CRASH_REPORT=0`; the module
rejects UART/crash-console builds.

For SIDE, the barcode sensor must sit over the bars, **more than 29 mm from the
line centre** on the barcode side (9 mm half-line + 20 mm gap, plus placement
margin). The remaining two sensors sit just inside opposite edges of the 18 mm
line. Both black means centred; left only requests left correction; right only
requests right correction; both white means line absent. Scanner transitions
cannot create a line/junction event or steering correction. Tracking and line-loss
detection continue while scanning. If barcodes are on the other side, reposition
the scanner or revise the input assignment after confirming the actual wiring.

Cross-junctions in SIDE: a crossing line covers all three sensors, but the two
edge sensors are also both black on a plain straight line, so the scanner's
black run length decides. A junction is reported once when all three are
black and the scanner has been black for 4.5 narrow widths of the last
decoded barcode (18 mm crossing is about 6 units, a wide bar 3). Inside a
frame the frame's own first (narrow) bar is the reference; before any
barcode a fixed `LINE_BARCODE_JUNCTION_MS` (270 ms, about 50 mm/s) is used.
Keep speed steady between a barcode and its junction. Limits: a sharp bend
that swings the line under the scanner, or a T-junction that only extends to
the non-scanner side, will not be classified correctly; verify on the track.
TRACK uses all three sensors for line position and junction candidates
(101/111), and disables barcode capture.

At power-up the service samples but remains **paused**:

1. Adjust each sensor comparator to distinguish track black from white.
2. Record each input's raw level on black, then white, as a
   `line_barcode_sensors_t` (`true` = pin reads 1); every sensor must change.
3. Call `line_service_calibrate(black_raw, white_raw)` while stationary.
   Measured active-low sensors would use `(0u, 7u)`; polarity is not assumed.
4. While paused, select the mounting profile; letter meanings may be set any time.
5. Call `line_service_enable(true)` when the controller is ready.

Example configuration (letters come from the MQTT mapping message):

```c
line_service_set_layout(LINE_BARCODE_LAYOUT_SIDE);
line_service_set_mapping('A', 'B', 'C', 'Z'); /* left, right, straight, U */
/* Check each ER result, then enable after measured calibration. */
```

Layout changes require PAUSED; `line_service_clear_map()` removes all
meanings. Calibration clears any stored letter; pause/resume keeps it. Mode reset does not erase agreed mappings. Changing layout resets
partial frames and recovery timers. No ADC calibration is claimed for digital
comparator outputs.

## Vehicle Controller interface

The controller implementation is absent from this repository. It must call the
APIs in `line_service.h` from its existing task; `subsystem3_init/start` already
create/start the service.

Every event/snapshot is a `line_barcode_msg_t` struct (see `line_barcode.h`):

| Field | Type | Meaning |
|---|---|---|
| `sensors` | 3 × `bool` | left/centre/right, true = black |
| `position` | `line_barcode_position_t` | far left, left, centre, right, far right, junction, none |
| `state` | `line_barcode_state_t` | FOLLOW, SEARCH, JUNCTION, SCANNING, PAUSED, FAULT |
| `event` | `line_barcode_event_t` | STATUS, JUNCTION, COMMAND, INVALID, LOST, FOUND, FAULT, BARCODE |
| `command` | `line_barcode_cmd_t` | LEFT, RIGHT, STRAIGHT, UTURN; NONE except on COMMAND |
| `fault` | `line_barcode_fault_t` | NONE, LINE, QUEUE, TIMING |
| `letter` | `char` | 'A'..'Z' on BARCODE/COMMAND, otherwise '\0' |

- Read `line_service_snapshot(&msg)` every control cycle. Check the return code:
  `E_TMOUT` means no sampling heartbeat in 30 ms; stop motion.
- Drain `line_service_receive(&msg, TMO_POL)` for events. `g_h_line_nav_q` is the
  architecture's `line_nav_q`, implemented with a micro T-Kernel message buffer.
  These are copied messages, not mailbox pointers. One controller task consumes.
- The letter is `msg.letter`. BARCODE carries no movement demand;
  it only says which letter is now stored. COMMAND is sent **at the junction**
  and includes the stored letter and its configured movement demand.
- Use current position for Buddy 2's wheel corrections. During absence the
  service retains the last useful steering direction in the position field;
  raw track bits still show absence. SEARCH starts after 100 ms without a line;
  FOUND reports reacquisition. After 2 s of search, LINE fault latches.
- PAUSED/FAULT must not produce line-following movement. Inspect state/fault
  before acting on a barcode, even if its letter decodes successfully. Obstacle
  avoidance and emergency stop take precedence. SCANNING means keep tracking
  the separate line at a slow scan speed, not drive blindly through line loss.
- Pause for encoder turns/avoidance with `line_service_enable(false)`; it drains
  queued events. Discard any command already copied into controller-local state.
  Resume with `true`; partial capture and loss timers reset. Buddy 5/controller
  must enforce the **50 cm** recovery limit with odometry; S3 has no distance
  feedback and does not pretend a time limit is a distance limit.
- Snapshot faults and queue overflow latch until explicit calibration/mode reset.
  Check the latest snapshot before applying queued events. Stop and reset after
  overflow; do not execute stale queued turns. INVALID provides no command.

Only the owning task mutates its processing context and reads IR GPIO. A priority
inheritance mutex protects packed configuration, mapping and snapshots. Queue
sends never wait. Per-input two-sample debouncing prevents barcode edges from
delaying the tracking inputs.

## Timing and printing limits

The service uses a 10 ms cyclic wake-up because `CNF_TIMER_PERIOD=10` and a task
delay adds a tick. Elapsed intervals over 20 ms invalidate capture and latch a
timing fault. Measure scheduling jitter with all buddy tasks running.

Widths must be 40–1000 ms, with three clearly wide elements in each character.
Classification is relative within a character, permitting different steady scan
speeds and gradual speed changes. Arbitrary acceleration within a character is
not supported. Both inter-character gaps are checked; each frame needs leading
and trailing quiet zones at least ten narrow widths. The supplied PDF crops very
close to its bars, so retain/add surrounding white paper when printing it.

At 3:1 wide:narrow, a 29-run frame occupies 47 narrow units. If its actual printed
bar span is 141 mm, the inferred narrow width is about 3 mm. Starting at 50 mm/s
therefore gives 60 ms per narrow element. This is an initial bench setting, not a
validated robot speed. Scan speed must match actual print dimensions; the smaller
PDF versions cannot be assumed to have the same timing. Trailing-zone validation
adds ten narrow widths of travel after the final bar (about 30 mm at that scale).
Ensure command placement leaves enough distance to the relevant junction.

## Data representation (before the change requirement)

| Data | Type in this version | Size on Pico (measured, `arm-none-eabi-nm`) |
|---|---|---|
| Event / snapshot message | struct: 3 `bool` + 5 `enum` + `char` | 9 bytes (`g_latest`) — 28 bytes on a 4-byte-enum host |
| Letter map | 26 × {`bool`, `enum`} | 52 bytes (`g_letter_map`) |
| Stored letter | `bool` + `char` | 2 bytes (`g_nav`) |
| Control flags | 5 separate `bool` | 5 bytes |
| Calibration levels | 3 `bool` | 3 bytes |
| Decoder context (task stack) | `uint16_t runs[29]`, `uint32_t` timers, `bool` flags | 108 bytes on host |
| line_nav_q buffer (kernel heap) | 12 × (message + 4) | 156 bytes |

The enums are 1 byte each on the Pico because the firmware toolchain packs
enums to the smallest type that fits; a host compiler uses 4 bytes. The packed
(after) version is backed up outside the repository in
`../subsystem3_AFTER_backup_2026-10-26/`.

## Build and verification

```sh
make -C build_make PICO_SDK_PATH="$HOME/pico-sdk" TEST=1 -j8
```

Flash `build_make/mtk3pico_smp1_usb_cdc_wifi_test.uf2` for on-target tests.
Portable host tests, including the supplied A/Z fixtures:

```sh
cc -std=c11 -Wall -Wextra -Werror -pedantic \
  -fsanitize=address,undefined -DAPP_TEST=1 -DLINE_BARCODE_HOST_TEST \
  app_program/subsystem3/line_barcode.c \
  app_program/subsystem3/line_barcode_test.c -o /tmp/subsystem3-tests
/tmp/subsystem3-tests
```

See `testing/subsystem3/TEST_LOG.md` and its saved logs. Successful host tests and
firmware builds do not prove physical mounting, polarity, tracking performance,
recognition rate, timing, or integrated navigation. Those remain hardware tests.

Measured linker comparison, same normal-image options and toolchain:

| Image | Text | Data | BSS | Static RAM (data+BSS) |
|---|---:|---:|---:|---:|
| Starter baseline `83664de` | 285124 | 88 | 24476 | 24564 bytes |
| Sample-matched implementation | 288804 | 92 | 24500 | 24592 bytes |

This is a 28-byte static RAM increase while adding the subsystem, not a measured
unpacked-to-packed saving for the same functionality. Runtime allocations also
include the 2048-byte task stack, kernel objects and 96-byte event buffer from
the existing heap. The 48-byte context lives on the stack. Hardware stack/heap
high-water and timing measurements remain pending. See the sample-revision host
and build logs; older provisional logs remain unchanged as historical evidence.
