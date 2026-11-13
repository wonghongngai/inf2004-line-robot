# Testing

All test records for the project live in this folder. A test only counts as
done when it is **written down here with its result and a saved log**.

```
testing/
├── README.md              this guide
├── subsystem1/            one folder per subsystem, kept by its owner
│   ├── TEST_LOG.md        test cases + results
│   └── logs/              saved console output, photos, scope captures
├── ...
├── subsystem5/
├── integration/           tests across two or more subsystems (shared)
└── system/                tests of the whole product against its requirements (shared)
```

**What to test, per buddy, and in what order: see [TEST_PLAN.md](TEST_PLAN.md).**

## Three levels of test

| Level | ID format | Tests what | Written by | Where |
|---|---|---|---|---|
| Unit | `S1-U01`, `S2-U01`, … | one subsystem on its own | the subsystem owner | `subsystemN/TEST_LOG.md` |
| Integration | `INT-01`, … | an interface between subsystems (TEAM.md section 5) | the owners on both sides | `integration/TEST_LOG.md` |
| System | `SYS-01`, … | the finished product against a requirement | whole team | `system/TEST_LOG.md` |

Test IDs never change and are never reused, even if a test is dropped.

## How to run and record a test

1. **Define it first.** Add a row to the *Test cases* table in the right
   `TEST_LOG.md`: ID, what it checks, how, and the expected result.
2. **Automate it if you can.** Add a `TEST_CHECK` with the same ID to
   `app_program/subsystemN/subsystemN_test.c` (see `app_program/test_report.h`).
   A test that needs eyes or instruments (an LED blinks, a motor turns, a
   scope shows 1 kHz) is a *manual* test. Write its steps in the table instead.
3. **Build the test image and flash it.**
   ```sh
   cd build_make
   make TEST=1 -j8        # -> mtk3pico_smp1_usb_cdc_wifi_test.uf2
   ```
   The test image runs every subsystem's tests at power-up and prints one
   `[TEST] <ID> PASS|FAIL` line per check. (A normal `make` leaves the tests out.)
4. **Save the console output** to `logs/`, named `YYYY-MM-DD_<what>.txt`:
   ```sh
   # macOS (find the port name with: ls /dev/cu.usbmodem*)
   cat /dev/cu.usbmodem1101 | tee ../testing/subsystem1/logs/2026-10-14_unit.txt
   # Linux / WSL with usbipd
   cat /dev/ttyACM0 | tee ../testing/subsystem1/logs/2026-10-14_unit.txt
   ```
   Start the command first, then reset the Pico. Press Ctrl-C when it is done.
   On Windows with PuTTY, use *Session → Logging → All session output*.
   For manual tests, save photos or scope screenshots in `logs/` the same way
   (keep each file under a few MB). Videos go in `videos/` instead (see below).
5. **Record the result.** Add a row to the *Results* table: date, test ID,
   the commit you tested (`git rev-parse --short HEAD`), who ran it,
   PASS/FAIL, and the log file. Commit the log and the table together.

## Host (PC) tests

Logic that doesn't need the Pico (packing, decoding, controllers) is tested on
the PC. One script builds and runs all of them:

```sh
sh testing/run_host_tests.sh              # every host test
sh testing/run_host_tests.sh s3/          # only subsystem 3
sh testing/run_host_tests.sh -v int/      # integration tests, full output
```

**Known failures.** A test that is broken for a tracked reason (e.g. code not
pushed yet) is listed in `known_failure()` in `testing/run_host_tests.sh` with
its issue number. It still runs and shows as `KNOWN`, but does not fail the
run; any other failure does. Remove its line once it passes (the summary tells
you). `--strict` treats known failures as failures too.

**On GitHub.** The same script runs automatically on every pull request and
every push to `main` (`.github/workflows/host-tests.yml`). The PR shows a
**Host tests** check; its full output is attached to the run as
`host-tests-output`. On GitHub's Linux machines the tests also run with
AddressSanitizer and UndefinedBehaviorSanitizer, so they can catch memory bugs
that a Windows build misses.

It needs a C compiler (`cc`: MSYS2/MinGW on Windows, or Linux/macOS/WSL) and,
for the two Python tests, `python3` or `python`. Each test is named
`<area>/<what>` (`s1`..`s5`, `int`). The script keeps going after a failure,
prints a PASS/FAIL summary with the commit, and exits non-zero if anything
failed. **Run it before opening a pull request.**

To record a run, save the output to the right `logs/` folder and add a
Results row for each test ID it covers:

```sh
sh testing/run_host_tests.sh | tee testing/integration/logs/2026-11-13_host.txt
```

To add a host test, add one `run_c` (or `run_py`) line to the list in
`testing/run_host_tests.sh`, with its build flags. Don't copy build commands
into READMEs; point to the script instead. Never commit the built `.exe`
files (they are gitignored).

Raw serial captures (`*_console.txt` from `serial_monitor.ps1`) are gitignored
because they can be tens of MB. Commit a trimmed copy of the runs a Results row
cites, saved under a different name (for example `2026-11-08_S5-U06.txt`).

Host tests check logic only. They never replace a hardware test.

## Videos of tests and demos

A manual test (an LED, a motor, a sensor reacting) is best shown with a short
video. Record 15–60 s showing the test ID and the hardware, share it as an
**unlisted YouTube** link (the only way videos are shared), list it in
**`videos/README.md`**, and put the link in the *Log / evidence* column of
the test's result row. `videos/README.md` explains how.

## Rules

- **Never edit or delete an old result.** If a test fails, record the FAIL,
  fix the code, re-run, and add a new PASS row. The history is the evidence.
- **Always record the commit.** A result is meaningless without the exact
  code it was run on. Commit your work *before* testing.
- **A FAIL needs a note** saying what went wrong and, later, which commit fixed it.
- Before merging a change into `main`, re-run your subsystem's unit tests.
  Before a demo or hand-in, run every test and record the full set.
