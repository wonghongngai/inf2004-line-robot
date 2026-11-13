#!/bin/sh
# run_host_tests.sh - build and run every PC-side (host) test in one go.
#
# Usage, from anywhere in the repo:
#   sh testing/run_host_tests.sh            run every host test
#   sh testing/run_host_tests.sh s3         only tests whose name contains "s3"
#   sh testing/run_host_tests.sh -v int/    also print each test's full output
#
# Save the output as evidence for a TEST_LOG row, for example:
#   sh testing/run_host_tests.sh | tee testing/integration/logs/2026-11-13_host.txt
#
# Every test is compiled and run even if an earlier one fails. The script exits
# 0 only when every selected test passes. Host tests check logic only: they
# never prove anything about the real hardware.
#
# Adding a test: add one `run_c` or `run_py` line in the list below, using the
# same build flags you would type by hand. Name it <area>/<what>, where <area>
# is s1..s5 or int, and give it a test ID in that area's TEST_LOG.md.

set -u

ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT" || exit 2

CC=${CC:-cc}
VERBOSE=0
FILTER=
for arg in "$@"; do
    case "$arg" in
        -v|--verbose) VERBOSE=1 ;;
        -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
        *) FILTER=$arg ;;
    esac
done

WORK=$(mktemp -d 2>/dev/null || mktemp -d -t hosttests)
trap 'rm -rf "$WORK"' EXIT INT TERM

PYTHON=
for p in python3 python; do
    if command -v "$p" >/dev/null 2>&1 && "$p" -c 'import sys' >/dev/null 2>&1; then
        PYTHON=$p
        break
    fi
done

# Use the strongest sanitizer this compiler can both build and run.
# MinGW (Windows) usually supports none; Linux/macOS/WSL get ASan + UBSan.
SAN=
printf 'int main(void){return 0;}\n' > "$WORK/probe.c"
for s in "-fsanitize=address,undefined" "-fsanitize=undefined"; do
    if $CC $s "$WORK/probe.c" -o "$WORK/probe" >/dev/null 2>&1 \
       && "$WORK/probe" >/dev/null 2>&1; then
        SAN=$s
        break
    fi
done

PASS=0
FAIL=0
SKIP=0
RESULTS=

record() {   # record <PASS|FAIL|SKIP> <name> <detail>
    RESULTS="$RESULTS$(printf '%-6s %-28s %s' "$1" "$2" "$3")
"
    case "$1" in
        PASS) PASS=$((PASS + 1)) ;;
        FAIL) FAIL=$((FAIL + 1)) ;;
        SKIP) SKIP=$((SKIP + 1)) ;;
    esac
}

selected() {
    [ -z "$FILTER" ] && return 0
    case "$1" in *"$FILTER"*) return 0 ;; esac
    return 1
}

show_tail() {   # show_tail <file>
    if [ "$VERBOSE" = 1 ]; then
        cat "$1"
    else
        tail -n 15 "$1"
    fi
    echo
}

run_c() {   # run_c <name> <compiler args...>  (sources, flags; -o is added)
    name=$1
    shift
    selected "$name" || return 0
    exe="$WORK/$(echo "$name" | tr '/' '_')"
    log="$exe.log"
    printf '%-28s ' "$name"
    if ! $CC $SAN "$@" -o "$exe" > "$log" 2>&1; then
        echo "FAIL (compile)"
        show_tail "$log"
        record FAIL "$name" "compile error"
        return 0
    fi
    if "$exe" > "$log" 2>&1; then
        echo "PASS"
        [ "$VERBOSE" = 1 ] && cat "$log"
        record PASS "$name" ""
    else
        echo "FAIL (run, exit $?)"
        show_tail "$log"
        record FAIL "$name" "assertion/run error"
    fi
}

run_py() {   # run_py <name> <script>
    name=$1
    selected "$name" || return 0
    printf '%-28s ' "$name"
    if [ -z "$PYTHON" ]; then
        echo "SKIP (no python)"
        record SKIP "$name" "python not found"
        return 0
    fi
    log="$WORK/$(echo "$name" | tr '/' '_').log"
    if CC=$CC "$PYTHON" "$2" > "$log" 2>&1; then
        echo "PASS"
        [ "$VERBOSE" = 1 ] && cat "$log"
        record PASS "$name" ""
    else
        echo "FAIL (exit $?)"
        show_tail "$log"
        record FAIL "$name" "python test failed"
    fi
}

echo "Host tests  (compiler: $CC, sanitizers: ${SAN:-none}, python: ${PYTHON:-none})"
echo "commit: $(git rev-parse --short HEAD 2>/dev/null || echo unknown)$(git diff --quiet HEAD 2>/dev/null || echo ' + uncommitted changes')"
echo

STRICT="-std=c11 -Wall -Wextra -Werror"
S1=app_program/subsystem1
S3=app_program/subsystem3
INT=testing/integration

# ---- Subsystem 1: Wi-Fi / MQTT / telemetry ---------------------------------
run_c  s1/protocol            $STRICT -O2 -I$S1 testing/subsystem1/protocol_test.c $S1/communication_protocol.c
run_c  s1/barcode_queue       $STRICT testing/subsystem1/barcode_queue_test.c $S1/communication_protocol.c
run_py s1/binary_cli          testing/subsystem1/test_binary.py

# ---- Subsystem 2: motors, encoders, motion protocol ------------------------
run_c  s2/motion_protocol     $STRICT -DS2_PROTOCOL_HOST_TEST=1 app_program/subsystem2/motion_protocol_host_test.c
run_c  s2/motor_pwm           $STRICT -Iinclude $INT/motor_pwm_host_test.c

# ---- Subsystem 3: line following + barcode ---------------------------------
run_c  s3/line_barcode        $STRICT -pedantic -DAPP_TEST=1 -DLINE_BARCODE_HOST_TEST $S3/line_barcode.c $S3/line_barcode_test.c
run_c  s3/pptx_samples        -std=c11 -DAPP_ALL_TEST=1 -I$S3 testing/subsystem3/pptx_samples_test.c $S3/line_barcode.c
run_c  s3/count_a             -std=c11 -DAPP_ALL_TEST=1 -DLINE_BARCODE_COUNT_A=1 -I$S3 testing/subsystem3/count_a_test.c $S3/line_barcode.c

# ---- Subsystem 4: IMU / humps ----------------------------------------------
run_c  s4/hump_height         $STRICT $INT/hump_height_host_test.c

# ---- Integration (two or more subsystems) ----------------------------------
run_c  int/robot_control      $STRICT -DAPP_ROBOT_INTEGRATED=1 -DAPP_TEST=0 -DS2_MOTION_DEMO=0 \
                              -DS2_LINE_CALIBRATION_READY=1 -include $INT/robot_host_stubs.h \
                              $INT/robot_host_test.c app_program/subsystem2/motion_control.c
run_c  int/line_service       $STRICT $INT/line_service_host_test.c $S3/line_barcode.c
run_c  int/all_buddies        $STRICT $INT/all_buddies_host_test.c $S1/communication_protocol.c
run_c  int/all_buddies_cont   $STRICT $INT/all_buddies_continuous_host_test.c $S1/communication_protocol.c
run_c  int/all_buddies_corner $STRICT $INT/all_buddies_corner_host_test.c $S1/communication_protocol.c
run_c  int/all_buddies_coll   $STRICT $INT/all_buddies_collision_host_test.c $S1/communication_protocol.c
run_c  int/all_buddies_white  $STRICT $INT/all_buddies_white_host_test.c $S1/communication_protocol.c
run_c  int/all_buddies_bal    $STRICT $INT/all_buddies_balance_host_test.c
run_py int/steering_ramp      $INT/test_steering_ramp.py

echo
echo "================ summary ================"
printf '%s' "$RESULTS"
echo "-----------------------------------------"
echo "$PASS passed, $FAIL failed, $SKIP skipped"
if [ $((PASS + FAIL + SKIP)) -eq 0 ]; then
    echo "No test name contains '$FILTER'."
    exit 2
fi
[ "$FAIL" -eq 0 ]
