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
after braces were added (fix 1). Scan output:

- before formatting: [`testing/integration/logs/2026-11-24_barr_c_scan.txt`](testing/integration/logs/2026-11-24_barr_c_scan.txt)
- after formatting: [`testing/integration/logs/2026-11-24_barr_c_scan_formatted.txt`](testing/integration/logs/2026-11-24_barr_c_scan_formatted.txt)
- after braces: [`testing/integration/logs/2026-11-24_barr_c_scan_braces.txt`](testing/integration/logs/2026-11-24_barr_c_scan_braces.txt)

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

## Verdict

The code **partly conforms**. The structural rules that matter most for
safety already hold everywhere: include guards, no `#include` of `.c` files,
no `auto` or `register`, no `setjmp`/`longjmp`/`exit`, lowercase function
names, fixed-width types for packed data, and a short interrupt handler that
shares only `volatile`, single-writer data. Subsystems 3 and 5 were written
to BARR-C from the start and are close to it.

The formatting and braces passes fixed almost all of the layout gaps
(brace placement 765 → 8, long lines 443 → 48), and every control-statement
body now has braces (217 → 0), the one finding with a real bug risk. The
firmware image did not change. What is left:

- **Naming prefixes** (`g_`, `p_`, `b_`), mainly in subsystem 2, subsystem
  4 and `all_test`.
- **The build enables no compiler warnings** on the RP2040 (fix 7).

## Results by rule

"Hits" counts lines (or functions, for 6.2). "Files" is how many of the 65
files have at least one hit, after both passes. "Before" is `deb7ade`;
"After" is after the formatting and braces passes.

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
| 3.5 | Tab characters | 238 | 104 | 10 | Nearly conforms: the rest are inside comments of the starter files |
| 3.6 | CRLF line endings in the repository | 0 | 0 | 0 | Conforms |
| 4.2 | Header without an include guard | 0 | 0 | 0 | Conforms |
| 4.3 | `#include` of a `.c` file | 0 | 0 | 0 | Conforms |
| 5.1 | `typedef` name not ending in `_t` | 40 | 40 | 16 | Deviation proposed (D2) |
| 5.2 | Plain `int` / `INT` / `UINT` | 279 | 279 | 34 | Deviation proposed (D1) |
| 5.2 | `short` or `long` keyword | 50 | 57 | 6 | Deviation proposed (D1): `printf` casts |
| 6.1 | Function name with uppercase | 0 | 0 | 0 | Conforms |
| 6.1 | Macro name with lowercase | 1 | 1 | 1 | Fix (`read_width` in `line_barcode.c`) |
| 6.2 | Function body over 100 lines | 18 | 20 | 14 | Deviation proposed (D4) |
| 6.2 | More than one `return` | 151 | 151 | 26 | Deviation proposed (D3) |
| 6.3 | Function-like macro | 17 | 17 | 8 | Partial: mostly register-address and test macros |
| 6.5 | Interrupt handler naming | 1 | 1 | 1 | Fix (`encoder_inthdr` → `encoder_isr`) |
| 7.1 | Variable name under 3 characters | 140 | 144 | 28 | Partial: mostly `er`, `i`, `t`, `l`, `r` |
| 7.1 | Variable name with uppercase | 0 | 0 | 0 | Conforms |
| 7.1 | File-scope variable without `g_` | 55 | 54 | 10 | Partial (S5 nearly conforms: 1) |
| 7.1 | Pointer variable without `p_` | 26 | 26 | 7 | Partial (S3, S5 conform) |
| 7.1 | Boolean variable without `b_` | 69 | 69 | 12 | Partial (S5 conforms) |
| 8.2 | `else if` chain without a final `else` | 21 | 21 | 7 | **Fix** (fix 2) |
| 8.3 | `switch` without `default` | 1 | 1 | 1 | Fix (`all_buddies_test.c`, `switch (cmd.bytes[0])`) |
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

- **Interrupt handler (6.5).** The only team ISR is `encoder_inthdr` in
  `subsystem2/drv_encoder.c`. It is `static`, short (clear edges, read GPIO,
  update two counters) and makes no blocking calls. Fix: BARR-C asks for ISR
  names to end in `_isr`.
- **Data shared with the ISR (1.8).** `ENCODER_CHANNEL.count` and `.zero`
  are `volatile int32_t`. Each has one writer (the ISR writes `count`, tasks
  write `zero`), and readers take `count - zero`, so no lock is needed for
  32-bit aligned access. Conforms.
- **Data shared between tasks.** Subsystem 5's cross-task words
  (`g_s5_request`, `g_s5_obstacle`, `g_s5_faults`) are `static volatile UW`
  with the `g_` prefix. Subsystems 1 to 4 share data through T-Kernel
  mutexes, event flags and message buffers instead. Conforms.
- **`robot_control.c`**: `volatile uint32_t s2_bench_stop_ms` is a
  non-`static` global used by the hardware bench test. Fix: add the `g_`
  prefix, or build it only when `TEST=1`.
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
| D5 | 8.6 | Constant on the right of `==` and `!=` | The rule exists to catch `if (x = 5)`. GCC's `-Wparentheses` (part of `-Wall`) catches that too, **but only once fix 7 turns it on**: the RP2040 build has no `-W` flags today. Changing 520 comparisons by hand risks typos for little gain. Subsystem 5 already uses constant-left |
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
2. **Add a final `else` (8.2)** to the 21 `else if` chains, even if it only
   holds a comment, and a `default:` to the one `switch` without one.
3. **Rename** `encoder_inthdr` → `encoder_isr` (6.5),
   `read_width` → `READ_WIDTH` or the function it aliases (6.1), and
   `s2_bench_stop_ms` → `g_s2_bench_stop_ms` (7.1).
4. **Tabs in comments (3.5)** in the starter files (`app_main.c`,
   `app_log.h`, `test_report.h`, `subsystem3*.c/h`, `subsystem5*.c/h`):
   replace with spaces.
5. ~~**Run clang-format** over `app_program/`.~~ Done 24 Nov, firmware
   unchanged (see "Formatting pass").
6. **Naming prefixes (7.1)** `g_`, `p_`, `b_` and names of 3 or more
   characters: new code only, unless an owner wants to rename their own
   files. Subsystems 3 and 5 can be used as the reference style.
7. **Turn on compiler warnings for `app_program/`.** `pico_rp2040.mk` builds
   with no `-W` flags (only the unused RX231 makefiles have `-Wall`), and
   `build_make/build_integrated.log` has 0 warnings only because none are
   enabled. Add `-Wall -Wextra` for the app objects only, count what it
   reports, then fix or record each kind. D5 depends on this.

## Team sign-off

Close #60 once each owner has read their subsystem's rows and the team has
agreed D1 to D7 (record the result in `DECISIONS.md`). Owners: the
formatting pass touched every file you own, so pull before editing.

- [ ] Buddy 1: subsystem 1
- [ ] Buddy 2: subsystem 2, D2, D4
- [ ] Buddy 3: subsystem 3
- [ ] Buddy 4: subsystem 4
- [ ] Buddy 5: subsystem 5
- [ ] Team: D1 to D7 agreed or rejected; fix list owners chosen
