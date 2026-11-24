# BARR-C coding standard review (#60)

**Why:** the project write-up (p1) says the code must conform to the Barr
Group *Embedded C Coding Standard* (BARR-C:2018), and code quality is
assessed. Until now no review was recorded (`DEVIATIONS.md` P3).

**What this is:** a measured review of the team's firmware against the rules
a script can count, plus a manual check of the rules it can't. It lists what
conforms, what doesn't, and proposes which gaps to fix and which to record as
deviations. **The team has not reviewed it yet** (see the sign-off list at
the end).

**Reviewed:** commit `deb7ade` (`main`, 24 Nov 2026), then again after
`clang-format` was run over `app_program/` the same day (fix 5 below), and
after braces were added (fix 1), and after fixes 2, 3, 4 and 7. Scan output:

- before formatting: [`testing/integration/logs/2026-11-24_barr_c_scan.txt`](testing/integration/logs/2026-11-24_barr_c_scan.txt)
- after formatting: [`testing/integration/logs/2026-11-24_barr_c_scan_formatted.txt`](testing/integration/logs/2026-11-24_barr_c_scan_formatted.txt)
- after braces: [`testing/integration/logs/2026-11-24_barr_c_scan_braces.txt`](testing/integration/logs/2026-11-24_barr_c_scan_braces.txt)
- after fixes 2, 3, 4 and 7: [`testing/integration/logs/2026-11-24_barr_c_scan_fixes.txt`](testing/integration/logs/2026-11-24_barr_c_scan_fixes.txt)
- after fix 6 in subsystems 2 and 4: [`testing/integration/logs/2026-11-24_barr_c_scan_naming_s2_s4.txt`](testing/integration/logs/2026-11-24_barr_c_scan_naming_s2_s4.txt) (includes the full rename list)
- after fix 6 in subsystems 1 and 3: [`testing/integration/logs/2026-11-24_barr_c_scan_naming_s1_s3.txt`](testing/integration/logs/2026-11-24_barr_c_scan_naming_s1_s3.txt) (includes the full rename list)
- after fix 6 in subsystem 5: [`testing/integration/logs/2026-11-24_barr_c_scan_naming_s5.txt`](testing/integration/logs/2026-11-24_barr_c_scan_naming_s5.txt)

## Scope

| In scope | Lines (after formatting) | Notes |
| --- | --- | --- |
| `app_program/subsystem1` to `subsystem5` | 16,451 | Team code in the `integrated` firmware, including each subsystem's `*_test.c` |
| `app_program/all_test` | 2,098 | Test harness only since 13 Nov (#4), so lowest priority |
| `app_program/app_main.c`, `app.h`, `app_log.h`, `test_report.h` | 402 | Shared start-up and test macros, mostly from the starter template |

Out of scope: `kernel/`, `include/`, `device/`, `lib/` (μT-Kernel and vendor
code, which has its own conventions; the root `.clang-format` disables
formatting there) and the host tests in `testing/` (built for the PC only,
never flashed).

## Method

1. **Script:** [`tools/barr_c_check.py`](tools/barr_c_check.py) strips
   comments and strings, then counts patterns for 28 checks across 17 rule
   sections. It is a heuristic line scanner, not a certified checker. Each
   count is a list of places to look, not a proven violation. I sampled the
   hits of every check by hand and removed the false-positive patterns I
   found (continuation-line alignment, parameter lists read as globals,
   `else if ... else` chains). A few false positives remain in most checks.
   Counts are per line, so a statement that formatting splits over two
   lines can count twice: small rises after formatting (8.6, 5.2, 7.1) are
   that, not new code.
2. **clang-format:** `app_program/.clang-format` approximates the Barr
   layout rules (4-space indent, braces on their own line, 80 columns).
3. **Manual review** of the rules a script can't judge: interrupt handlers,
   data shared with them, `volatile`, and naming.

Rule numbers below are BARR-C:2018 section numbers (for example 1.3 is
"Braces"). Please check any rule you want to quote in the report against the
standard itself.

Re-run with:

```sh
python tools/barr_c_check.py                 # summary
python tools/barr_c_check.py --files         # per-file counts
python tools/barr_c_check.py --show 1.3-nobrace   # every hit for one check
python tools/barr_c_check.py app_program/subsystem2   # one folder
```

## Formatting pass (fix 5, 24 Nov)

`clang-format` with `app_program/.clang-format` was run over every `.c` and
`.h` file in `app_program/` (56 files changed). Checked:

- **Whitespace only.** With all spaces, tabs and newlines removed, 41 files
  are identical to `deb7ade`. The other 15 differ only where whitespace is
  itself a token boundary: `*` spacing, macro `\` continuations and split
  call lines.
- **Same firmware.** `integrated` was built from `deb7ade` and from the
  formatted tree. The loadable images (`objcopy -O binary`) are identical
  (SHA-256 `aafa397d…a89b95`; text 393,004 B, data 120 B, bss 84,032 B in
  both).
- **Host tests:** `sh testing/run_host_tests.sh` passes 17 of 17.

After the pass, 51 of 65 files match the formatter (was 9). The other 14 have
69 small differences in total, because clang-format 15 does not always settle
in one run. The second run (fix 1) cleared them: all 65 files now match.

## Braces pass (fix 1, 24 Nov)

`InsertBraces: true` was added to `app_program/.clang-format` and the
formatter run again. It braced 216 of the 217 unbraced bodies. The last one
was in `motion_task()` (`subsystem2/task_motion.c`): an `else` whose body
came after an `#else ... #endif`, which clang-format can't see across. It was
rewritten by hand so each `#if` branch holds a complete statement:

```c
#if APP_ROBOT_INTEGRATED || S2_MOTION_DEMO
        if (manual_stop)
        {
            ...
        }
        else
        {
            robot_control_step(&controller, now_ms, left_ticks, right_ticks);
        }
#else
        (void)manual_stop;
        robot_control_step(&controller, now_ms, left_ticks, right_ticks);
#endif
```

Checked against the formatted tree before braces (`objcopy -O binary`):

| Build | Result |
| --- | --- |
| `integrated` | Identical (SHA-256 `aafa397d…a89b95`, the same as `deb7ade`) |
| `all_test` | Identical |
| `motion` (takes the `#else` branch of the hand edit) | Identical |
| `integrated TEST=1` | Same size; 722 bytes differ, all `__LINE__` values that `TEST_CHECK` prints, shifted by the new brace lines |
| Host tests | 17 of 17 pass |

The second run also re-aligned some `#define` columns across comments
(`AlignConsecutiveMacros: AcrossComments`), which pushed trailing comments
on 13 more lines past 80 columns (1.2: 35 → 48).

## Fixes 2, 3, 4 and 7 (24 Nov)

- **Fix 2 (8.2, 8.3).** Each of the 21 `else if` chains now ends in an
  `else` block that holds only the comment `/* No action needed in any other
  case. */`. The `switch (cmd.bytes[0])` in `all_buddies_test.c` has a
  `default: break;`: it was missing only `S1_COMMAND_REVERSE`, which
  `s1_command_valid()` accepts and the test loop ignores, as before.
- **Fix 3 (6.1, 6.5, 7.1).** `encoder_inthdr` → `encoder_isr`;
  `s2_bench_stop_ms` → `g_s2_bench_stop_ms` (`robot_control.c`,
  `subsystem2_hw_test.h`, `task_motion.c`); the `read_width` alias macro in
  `line_barcode.c` is gone, and its four callers call
  `line_barcode_run_width()` directly.
- **Fix 4 (3.5).** The 104 tabs, all inside comments of the starter files,
  are now spaces (tab width 4, as in `.clang-format`).
- **Fix 7 (compiler warnings).** The `app_program/` compile rule in
  `build_make/mtkernel_3/app_program/subdir.mk` now adds
  `APP_WARN ?= -Wall -Wextra`; kernel and vendor objects keep their own
  flags. With all 34 app objects rebuilt, `integrated`, `all_test`,
  `integrated TEST=1` and `motion` each give **0 warnings**. As a control,
  a test file with an unused variable and `if (a = b)` gives 2 warnings with
  the same toolchain and flags.

Checked: all four images (`integrated`, `all_test`, `integrated TEST=1`,
`motion`) are byte-identical to the braces commit, and the host tests pass
17 of 17.

## Fix 6 in subsystems 2 and 4 (24 Nov)

Every variable, local and parameter in `app_program/subsystem2/` and
`subsystem4/` now follows rule 7.1: 436 renames, done by script from a
Universal Ctags inventory, so each rename stays in its scope.

- **File-scope variables** take `g_` (`armed` → `g_b_armed`,
  `trace_ring` → `g_trace_ring`). `const` tables keep their names.
- **Pointers** take `p_`, **Booleans** take `b_` (`controller` →
  `p_controller`, `moving` → `b_moving`). Prototype parameter names in
  the headers were changed to match.
- **Names under 3 characters** got real names: `er` → `ercd` (the
  T-Kernel convention), `l`/`r` → `left_ticks`/`right_ticks`,
  `i` → `idx`, `p` → `p_arg`, `c` → `p_ctx`, and per-function names
  such as `dl` → `delta_left`, `ms` → `delay_ms`.
- **Shared kernel objects and S4's snapshot** are now `g_flg_motion_stop`,
  `g_mbf_motion_cmd`, `g_mtx_motion_status`, `g_mtx_imu_data` and
  `g_s4_imu_data`, also in `TEAM.md`, `DEVIATIONS.md` (A3) and
  `testing/integration/test_event_bits.py`. Old names stay in the test
  logs, which record what was true at the time.
- The host tests that `#include` `robot_control.c` and `task_imu.c` were
  renamed with them.

Checked: all 14 build variants (the 11 in `build.sh` plus
`integrated S4_TRACE=1` and the PID and pivot benches) build with 0
warnings, and 13 are byte-identical to the commit before. The 14th,
`integrated TEST=1`, differs in 5 bytes, each one higher: the `TEST_CHECK`
line numbers in `subsystem4_test.c`, which gained one wrapped line. Host
tests 17 of 17.

## Fix 6 in subsystems 1 and 3 (24 Nov)

The same approach, 140 renames: 18 file-scope statics in S1 (`client` →
`g_p_client`, the four queues → `g_..._queue`), pointer parameters such as
`packet` → `p_packet` and `black` → `p_black`, five Booleans, and short
names (`i` → `idx`; in `decode_character()` `i`/`j`/`k` → `elem`/`slot`/
`rank`; in `line_barcode_test()` `i` → `loop_idx`, as `idx` was taken).
Nothing outside S1 and S3 changed apart from the two host tests that
`#include` S1 sources.

**Rename tool fix.** The collision check in the S2/S4 pass never fired
(a broken regex). Re-run, fixed, against the S2/S4 sources before the
rename, it finds exactly one clash, `ev` → `event` in `straddle_step()`,
the one the host tests had already caught and that was fixed by hand. The
check reads the source, so it also covers `#if` branches no build
compiles. S1 and S3 ran with the fixed check.

Checked against the commit before any renames (S1 to S4 together): 14 of
15 build variants are byte-identical (the 14 above plus `integrated` with
`WIFI=none NET=none MQTT=0`). `integrated TEST=1` grows by 4 bytes, all in
`line_barcode_test.o`: its `TEST_CHECK` line numbers moved, and on the
Cortex-M0+ some of them now take a literal-pool load instead of a shifted
`movs` (592 is `148 << 2`; 593 is not). Compiled with `__LINE__` fixed at
0, the old and new `line_barcode_test.c` give identical code. 0 warnings;
host tests 17 of 17.

## Fix 6 in subsystem 5 (24 Nov)

Subsystem 5 was written with the prefixes, so it needed only 14 names:
`median3()`'s `a`/`b`/`c` → `first`/`second`/`third`, `servo_abs_diff()`'s
`a`/`b` → `lhs`/`rhs`, `mm`/`cm` → `dist_mm`/`dist_cm`, `us` → `delay_us`,
`in` → `input`, `on` → `probe`, `exinf` → `p_exinf`, and the event flag
`flg_s5_evt` → `g_flg_s5_evt` (also in two S2 host-test files,
`test_event_bits.py`, `TEAM.md`, `DEVIATIONS.md` and the S5 README).
Names such as `p_x` and `b_up` already meet 7.1, which counts the whole
name. The rename tool now only adds prefixes: its first S5 plan would have
removed the owner's `g_` from `const` tables and `b_` from a `UB` flag.
The S1 to S4 plans removed none.

Checked against the commit before any renames (S1 to S5 together): 14 of
15 build variants are byte-identical. In `integrated TEST=1` only the three
test runners (`line_barcode_test.o`, `subsystem4_test.o`,
`subsystem5_test.o`) differ, and each compiles to identical code with
`__LINE__` fixed. 0 warnings; host tests 17 of 17.

Only `all_test` (the test harness) was not renamed.

## Verdict

The code **partly conforms**. The structural rules that matter most for
safety already hold everywhere: include guards, no `#include` of `.c` files,
no `auto` or `register`, no `setjmp`/`longjmp`/`exit`, lowercase function
names, fixed-width types for packed data, and a short interrupt handler that
shares only `volatile`, single-writer data. Subsystems 3 and 5 were written
to BARR-C from the start and are close to it.

The formatting and braces passes fixed almost all of the layout gaps
(brace placement 765 → 8, long lines 443 → 48), and every control-statement
body now has braces (217 → 0), the one finding with a real bug risk. Every
`else if` chain ends in `else` and every `switch` has a `default`, and the
team code compiles with no warnings under `-Wall -Wextra`. The firmware
image did not change. What is left:

- **Naming prefixes** (`g_`, `p_`, `b_`) and short names in `all_test`
  only (fix 6 is done for subsystems 1 to 5).
- **Deviations D1 to D7**, which the team still has to agree.

## Results by rule

"Hits" counts lines (or functions, for 6.2). "Files" is how many of the 65
files have at least one hit, after both passes. "Before" is `deb7ade`;
"After" is after the formatting and braces passes and fixes 2, 3, 4 and 7.

| Rule | Check | Before | After | Files | Status |
| --- | --- | ---: | ---: | ---: | --- |
| 1.1 | C standard: `-std=gnu11` | — | — | — | Deviation proposed (D7) |
| 1.2 | Line longer than 80 characters | 443 | 48 | 19 | Nearly conforms: long string literals and aligned `#define` comments |
| 1.3 | Left brace not on a line by itself | 765 | 8 | 4 | Conforms: the 8 are compound literals, e.g. `(MOTION_COMMAND){...}` |
| 1.3 | `if`/`else`/`for`/`while` body without braces | 217 | 0 | 0 | Conforms (fix 1). The first two scan logs say 231: the script misread 14 `#if`/`#else` lines, fixed before the braces log |
| 1.7 | `goto` | 16 | 16 | 3 | Deviation proposed (D6): 10 in `usermain()`, 6 in `all_test` |
| 1.7 | `continue` | 13 | 13 | 7 | Partial |
| 1.7 | `auto` or `register` | 0 | 0 | 0 | Conforms |
| 3.4 | Indent not a multiple of 4 | 7 | 9 | 5 | Conforms (hits are aligned continuation lines) |
| 3.5 | Tab characters | 238 | 0 | 0 | Conforms (fix 4) |
| 3.6 | CRLF line endings in the repository | 0 | 0 | 0 | Conforms |
| 4.2 | Header without an include guard | 0 | 0 | 0 | Conforms |
| 4.3 | `#include` of a `.c` file | 0 | 0 | 0 | Conforms |
| 5.1 | `typedef` name not ending in `_t` | 40 | 40 | 16 | Deviation proposed (D2) |
| 5.2 | Plain `int` / `INT` / `UINT` | 279 | 279 | 34 | Deviation proposed (D1) |
| 5.2 | `short` or `long` keyword | 50 | 57 | 6 | Deviation proposed (D1): `printf` casts |
| 6.1 | Function name with uppercase | 0 | 0 | 0 | Conforms |
| 6.1 | Macro name with lowercase | 1 | 0 | 0 | Conforms (fix 3: `read_width` alias removed) |
| 6.2 | Function body over 100 lines | 18 | 20 | 14 | Deviation proposed (D4) |
| 6.2 | More than one `return` | 151 | 151 | 26 | Deviation proposed (D3) |
| 6.3 | Function-like macro | 17 | 17 | 8 | Partial: mostly register-address and test macros |
| 6.5 | Interrupt handler naming | 1 | 0 | 0 | Conforms (fix 3: `encoder_isr`) |
| 7.1 | Variable name under 3 characters | 140 | 30 | 4 | Conforms in S1 to S5 (fix 6; the hits left there are union members); 24 in `all_test` |
| 7.1 | Variable name with uppercase | 0 | 0 | 0 | Conforms |
| 7.1 | File-scope variable without `g_` | 55 | 9 | 7 | Conforms (fix 6): the 9 left are `const` tables and prototype parameter lines |
| 7.1 | Pointer variable without `p_` | 26 | 11 | 2 | Conforms in S1 to S5; 11 in `all_test` |
| 7.1 | Boolean variable without `b_` | 69 | 12 | 2 | Conforms in S1 to S5; 12 in `all_test` |
| 8.2 | `else if` chain without a final `else` | 21 | 0 | 0 | Conforms (fix 2) |
| 8.3 | `switch` without `default` | 1 | 0 | 0 | Conforms (fix 2) |
| 8.5 | `setjmp`/`longjmp`/`abort`/`exit` | 0 | 0 | 0 | Conforms |
| 8.6 | `==`/`!=` with the constant on the right | 503 | 520 | 31 | Deviation proposed (D5) (S5 nearly conforms: 5) |

## Results by subsystem

Counts after formatting (before braces), for the checks with hits. "—" means none.

| Check | S1 (1,433 lines) | S2 (5,618) | S3 (2,375) | S4 (2,278) | S5 (4,747) | all_test (2,098) | start-up and macros (402) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1.2 width | 10 | 12 | 3 | 4 | 2 | 3 | 1 |
| 1.3 missing braces (now 0 everywhere) | 1 | 77 | 19 | 10 | — | 97 | 13 |
| 3.5 tabs (in comments) | — | — | 12 | — | 21 | — | 71 |
| 5.1 typedef `_t` | 8 | 16 | — | 10 | 4 | 2 | — |
| 5.2 `int` types | 92 | 56 | 6 | 21 | 63 | 36 | 5 |
| 6.2 over 100 lines | — | 7 | 2 | 2 | 4 | 4 | 1 |
| 6.2 multiple returns | 23 | 47 | 4 | 16 | 44 | 15 | 2 |
| 7.1 short names | 14 | 55 | 15 | 30 | 4 | 24 | 2 |
| 7.1 `g_` prefix | 10 | 40 | 2 | — | 1 | 1 | — |
| 7.1 `b_` prefix | — | 40 | 6 | 11 | — | 12 | — |
| 8.2 final `else` | 1 | 10 | 1 | 2 | — | 7 | — |
| 8.6 constant on right | 78 | 204 | 26 | 38 | 5 | 165 | 4 |

## Manual review

- **Interrupt handler (6.5).** The only team ISR is `encoder_isr` in
  `subsystem2/drv_encoder.c` (`encoder_inthdr` before fix 3). It is
  `static`, short (clear edges, read GPIO, update two counters) and makes no
  blocking calls. Conforms.
- **Data shared with the ISR (1.8).** `ENCODER_CHANNEL.count` and `.zero`
  are `volatile int32_t`. Each has one writer (the ISR writes `count`, tasks
  write `zero`), and readers take `count - zero`, so no lock is needed for
  32-bit aligned access. Conforms.
- **Data shared between tasks.** Subsystem 5's cross-task words
  (`g_s5_request`, `g_s5_obstacle`, `g_s5_faults`) are `static volatile UW`
  with the `g_` prefix. Subsystems 1 to 4 share data through T-Kernel
  mutexes, event flags and message buffers instead. Conforms.
- **`robot_control.c`**: `volatile uint32_t g_s2_bench_stop_ms` is a
  non-`static` global used by the hardware bench test, now with the `g_`
  prefix (fix 3).
- **Infinite loops.** 13 task loops; 10 already use `for (;;)` and 3 use
  `while (1)` or `while (true)`.
- **Casts (1.6).** BARR-C asks for a comment on each cast. Most casts are
  `(UINT)` or `(unsigned long)` for `printf` arguments, or `(FP)` when
  registering handlers. This was not counted; propose treating `printf`
  casts as covered by D1.

## Proposed deviations (team to agree)

These are places where the team's code breaks a rule on purpose. BARR-C
allows a project to deviate from a rule if the deviation is written down and
justified, so each one needs a team yes or no.

| ID | Rule | Deviation | Reason |
| --- | --- | --- | --- |
| D1 | 5.2 | T-Kernel types (`ER`, `ID`, `INT`, `UINT`, `BOOL`, `TMO`) at RTOS calls; `unsigned long` / `long` casts for `printf` `%lu` / `%ld` | The μT-Kernel API is declared with these types, and `printf` format specifiers need them. Packed and wire data already uses `uint8_t` to `uint32_t` |
| D2 | 5.1 | Existing uppercase typedef names (`MOTION_CONFIG`, `S1_TELEMETRY`, `IMU_MOTION_STATE`, …) | They match the T-Kernel naming used beside them (`T_DINT`, `SYSTIM`). Renaming 40 types across subsystem interfaces this close to the demo risks breaking integration. New types use `_t` |
| D3 | 6.2 | Early `return` on error | BARR-C states a single exit point as a preferred practice. Early returns keep the error paths short and avoid deep nesting |
| D4 | 6.2 | Functions over 100 lines: the `*_test()` runners and the task main loops (`imu_task`, `motion_task`, `obstacle_task`, `line_service_task`, `control_step`) | Test runners are flat lists of cases. Splitting the control loops now would need a full re-test of S2-U12 timing and the track runs (#18, #19) |
| D5 | 8.6 | Constant on the right of `==` and `!=` | The rule exists to catch `if (x = 5)`. GCC's `-Wparentheses` (part of `-Wall`) catches that too, and fix 7 turned `-Wall -Wextra` on for all team code (0 warnings today). Changing 520 comparisons by hand risks typos for little gain. Subsystem 5 already uses constant-left |
| D6 | 1.7 | Forward `goto` to the single `startup_failed` label in `usermain()` (10 uses) | Every jump goes forward to one error exit in the same function, the exception BARR-C allows. The 6 `goto` in `all_test` should go instead |
| D7 | 1.1 | `-std=gnu11` (`build_make/pico_rp2040.mk:238`), not strict C99 | Set once for the whole build, kernel and lwIP included. This review did not check the team code for GNU extensions |

## Proposed fixes

In priority order. Each is a mechanical change that the host tests and the
firmware comparison above can check. Owners are the subsystem buddies;
nothing has been assigned yet.

1. ~~**Add braces to every unbraced body (1.3).**~~ Done 24 Nov with
   `InsertBraces` plus one hand edit; firmware unchanged (see "Braces
   pass"). `InsertBraces` stays on, so new code gets braces whenever the
   formatter runs. clang-format's documentation warns it can be wrong
   because it lacks full semantic information: review its diff near `#if`
   lines.
2. ~~**Add a final `else` (8.2)** and a `default:` (8.3).~~ Done 24 Nov.
   The `else` blocks hold only a generic comment; owners may want to say
   why no action is needed in their own words.
3. ~~**Rename** `encoder_inthdr`, `read_width` and `s2_bench_stop_ms`.~~
   Done 24 Nov.
4. ~~**Tabs in comments (3.5).**~~ Done 24 Nov.
5. ~~**Run clang-format** over `app_program/`.~~ Done 24 Nov, firmware
   unchanged (see "Formatting pass").
6. ~~**Naming prefixes (7.1)** `g_`, `p_`, `b_` and names of 3 or more
   characters.~~ Done 24 Nov for subsystems 1 to 5. `all_test` is a test
   harness and was left as it is.
7. ~~**Turn on compiler warnings for `app_program/`.**~~ Done 24 Nov:
   `-Wall -Wextra` on the app compile rule, 0 warnings in four modes. Keep
   the build at 0: a new warning in a build log is now worth reading.
   Adding `-Werror` would enforce that, but would also stop a demo build
   for a harmless warning, so it is left for the team to decide.

## Team sign-off

Close #60 once each owner has read their subsystem's rows and the team has
agreed D1 to D7 (record the result in `DECISIONS.md`). Owners: the
formatting pass touched every file you own, so pull before editing.

Signed off 24 Nov 2026; D1 to D7 agreed (`DECISIONS.md`, 24 Nov).

- [x] Buddy 1: subsystem 1
- [x] Buddy 2: subsystem 2, D2, D4
- [x] Buddy 3: subsystem 3
- [x] Buddy 4: subsystem 4
- [x] Buddy 5: subsystem 5
- [x] Team: D1 to D7 agreed
