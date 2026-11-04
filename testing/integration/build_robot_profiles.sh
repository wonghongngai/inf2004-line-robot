#!/bin/sh
set -eu
cd "$(dirname "$0")/../../build_make"
: "${PICO_SDK_PATH:?Set PICO_SDK_PATH to the installed SDK}"
for profile in integrated_test motion motion_demo motor_test integrated_motor_test integrated_single integrated; do
    case "$profile" in
        integrated_test) set -- ROBOT_MODE=integrated TEST=1 ;;
        motion) set -- ROBOT_MODE=motion ;;
        motion_demo) set -- ROBOT_MODE=motion MOTION_DEMO=1 ;;
        motor_test) set -- ROBOT_MODE=motor_test ;;
        integrated_motor_test) set -- ROBOT_MODE=integrated_motor_test ;;
        integrated_single) set -- ROBOT_MODE=integrated SMP=0 ;;
        integrated) set -- ROBOT_MODE=integrated ;;
    esac
    if make -j8 "$@" > "robot_${profile}_build.log" 2>&1; then
        echo "$profile: build PASS"
    else
        tail -40 "robot_${profile}_build.log"
        exit 1
    fi
done
