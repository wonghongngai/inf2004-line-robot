#!/bin/sh
# Shared firmware build script (issue #11). Same command on every machine:
#
#     sh build.sh <mode> [VAR=value ...]
#     sh build.sh all [VAR=value ...]
#
# Needs only arm-none-eabi-gcc on PATH and PICO_SDK_PATH in the environment
# (README section 4). Runs from any folder. Extra VAR=value arguments go
# straight to make and override the mode's preset, e.g.
#
#     sh build.sh integrated TEST=1
#     sh build.sh all_test ALL_TEST_CONTINUE_ON_SENSOR=1
#
# Prints the path of the .uf2 it produced. The full make output is shown and
# also saved to build_make/build_<mode>[_<options>].log.
#
# 'all' builds every mode and test variant one after another and prints a
# PASS/FAIL/SKIP table (run it before opening a pull request).
set -eu

usage() {
	cat <<'EOF'
Usage: sh build.sh <mode> [VAR=value ...]
       sh build.sh all [VAR=value ...]

Modes:
  integrated             final robot firmware, Wi-Fi + MQTT (the demo build)
  all_test               whole-robot test harness, Wi-Fi + MQTT
  all_test_offline       all_test without Wi-Fi
  integrated_motor_test  wheels raised: motors + IR + IMU diagnostics
  motor_test             wheels raised: motors only
  motion                 S2 motion controller (add MOTION_DEMO=1 for the demo)
  line_follow_test       S2 + S3 open-loop line following
  barcode_test           S3 barcode bench, motors off
  all                    build every mode and test variant, print a summary

Every mode builds with TEST=0 CONSOLE=usb_cdc CRASH_REPORT=0 unless overridden.
EOF
}

# Every build 'all' runs: each mode, plus the test variants in README 2.1.
ALL_BUILDS='integrated
integrated TEST=1
all_test
all_test_offline
integrated_motor_test
motor_test
motion
motion TEST=1
motion MOTION_DEMO=1
line_follow_test
barcode_test'

if [ $# -lt 1 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
	usage
	[ $# -ge 1 ] && exit 0
	exit 2
fi

script=$0
mode=$1
shift

for arg in "$@"; do
	case $arg in
		*=*) ;;
		*) echo "build.sh: '$arg' is not VAR=value; only make variables may follow the mode" >&2; exit 2 ;;
	esac
done

# The preset make variables for a mode. Prints nothing for an unknown mode.
preset_for() {
	offline="WIFI=none NET=none MQTT=0"
	case $1 in
		integrated)       p="ROBOT_MODE=integrated WIFI=cyw43 NET=lwip MQTT=1" ;;
		all_test)         p="ROBOT_MODE=all_test WIFI=cyw43 NET=lwip MQTT=1" ;;
		all_test_offline) p="ROBOT_MODE=all_test ALL_TEST_OFFLINE=1 $offline" ;;
		integrated_motor_test | motor_test | motion | line_follow_test | barcode_test)
		                  p="ROBOT_MODE=$1 $offline" ;;
		*) return 0 ;;
	esac
	echo "TEST=0 CONSOLE=usb_cdc CRASH_REPORT=0 $p"
}

# Effective MQTT setting of a VAR=value list (later words win).
mqtt_of() {
	m=0
	for a in "$@"; do
		case $a in MQTT=*) m=${a#MQTT=} ;; esac
	done
	echo "$m"
}

# Log file for a mode and its extra options, e.g. build_motion_TEST=1.log.
log_for() {
	l=build_$1
	shift
	for a in "$@"; do
		l=${l}_$(printf '%s' "$a" | tr -c 'A-Za-z0-9=_.-' '_')
	done
	echo "$l.log"
}

if [ "$mode" != all ] && [ -z "$(preset_for "$mode")" ]; then
	echo "build.sh: unknown mode '$mode'" >&2
	echo >&2
	usage >&2
	exit 2
fi

if [ -z "${PICO_SDK_PATH:-}" ]; then
	echo "build.sh: PICO_SDK_PATH is not set. See README section 4.4, e.g." >&2
	echo '    export PICO_SDK_PATH="$HOME/pico/pico-sdk"' >&2
	exit 1
fi
case $PICO_SDK_PATH in
	*" "*)
		echo "build.sh: PICO_SDK_PATH contains a space: '$PICO_SDK_PATH'" >&2
		echo "make cannot handle that. Use the folder's short name (cmd /c dir /x C:\\Users)," >&2
		echo "e.g. C:/Users/FIRSTL~1/.pico-sdk/sdk/2.3.0, or move the SDK. See README section 4.4." >&2
		exit 1
		;;
esac

# Go to build_make/ (next to this script) wherever it was started from.
# Everything below is relative to this folder.
cd "$(dirname "$script")/build_make"
script=../build.sh

if [ "$mode" = all ]; then
	out=$(mktemp)
	trap 'rm -f "$out"' EXIT
	# Options given after 'all' are added to every build. They are re-split
	# on spaces below, so values containing spaces are not supported here.
	extra=$*
	pass=0 fail=0 skip=0 summary=""
	total=$(printf '%s\n' "$ALL_BUILDS" | wc -l | tr -d ' ')
	n=0
	# Read the list on fd 3 so the builds cannot consume it from stdin.
	while IFS= read -r spec <&3; do
		n=$((n + 1))
		name="$spec${extra:+ $extra}"
		# shellcheck disable=SC2086
		set -- $spec $extra
		m=$1
		shift
		# shellcheck disable=SC2086
		if [ "$(mqtt_of $(preset_for "$m") "$@")" = 1 ] && [ ! -f ../config/wifi_secrets.h ]; then
			result="SKIP  $name  (needs config/wifi_secrets.h)"
			skip=$((skip + 1))
			printf 'build.sh all: [%s/%s] %s ... skipped (no config/wifi_secrets.h)\n' "$n" "$total" "$name"
		else
			printf 'build.sh all: [%s/%s] %s ... ' "$n" "$total" "$name"
			start=$(date +%s)
			if sh "$script" "$m" "$@" > "$out" 2>&1 < /dev/null; then
				result="PASS  $name  -> $(sed -n 's/^Built: //p' "$out")"
				pass=$((pass + 1))
				echo "ok ($(( $(date +%s) - start ))s)"
			else
				result="FAIL  $name  (see build_make/$(log_for "$m" "$@"))"
				fail=$((fail + 1))
				echo "FAILED ($(( $(date +%s) - start ))s)"
			fi
		fi
		summary="$summary$result
"
	done 3<<EOF
$ALL_BUILDS
EOF
	echo
	echo "----------------------------------------------------------------"
	printf '%s' "$summary"
	echo "----------------------------------------------------------------"
	echo "$pass passed, $fail failed, $skip skipped"
	[ "$fail" = 0 ]
	exit
fi

preset=$(preset_for "$mode")

# shellcheck disable=SC2086
if [ "$(mqtt_of $preset "$@")" = 1 ] && [ ! -f ../config/wifi_secrets.h ]; then
	echo "build.sh: $mode needs config/wifi_secrets.h (Wi-Fi and MQTT broker settings)." >&2
	echo "Copy config/wifi_secrets.example.h to config/wifi_secrets.h and fill it in." >&2
	echo "Never commit it. Or build without Wi-Fi: add WIFI=none NET=none MQTT=0." >&2
	exit 1
fi

jobs=$(nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 4)
log=$(log_for "$mode" "$@")

echo "build.sh: make -j$jobs $preset $*"
# $preset is split on spaces on purpose; it holds only VAR=value words.
# A pipe hides make's exit status (POSIX sh has no pipefail), so it is
# passed out through a file.
# shellcheck disable=SC2086
{ make -j"$jobs" $preset "$@" 2>&1; echo $? > .build_status; } | tee "$log"
status=$(cat .build_status 2>/dev/null || echo 1)
rm -f .build_status
if [ "$status" != 0 ]; then
	echo >&2
	echo "build.sh: BUILD FAILED ($mode). Full output: build_make/$log" >&2
	exit "$status"
fi

# shellcheck disable=SC2086
exe=$(make -s $preset "$@" print-exe | tail -n 1)
if [ ! -f "$exe.uf2" ]; then
	echo "build.sh: make succeeded but build_make/$exe.uf2 is missing" >&2
	exit 1
fi

echo
echo "Built: build_make/$exe.uf2"
echo "Flash it: hold BOOTSEL, plug in the Pico, copy the .uf2 onto the RPI-RP2 drive."
