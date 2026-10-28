#!/bin/sh
set -eu
cd "$(dirname "$0")/../.."
test_binary=$(mktemp /tmp/robot-host-test.XXXXXX)
trap 'rm -f "$test_binary"' EXIT
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
  -DAPP_ROBOT_INTEGRATED=1 -DAPP_TEST=0 -DS2_MOTION_DEMO=0 \
  -DS2_LINE_CALIBRATION_READY=1 \
  -include testing/integration/robot_host_stubs.h \
  testing/integration/robot_host_test.c app_program/subsystem2/motion_control.c \
  -o "$test_binary"
"$test_binary"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
  testing/integration/all_buddies_host_test.c \
  app_program/subsystem1/communication_protocol.c -o "$test_binary"
"$test_binary"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined -Iinclude \
  testing/integration/motor_pwm_host_test.c -o "$test_binary"
"$test_binary"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
  testing/integration/hump_height_host_test.c -o "$test_binary"
"$test_binary"
cc -std=c11 -Wall -Wextra -Werror -fsanitize=undefined \
  testing/integration/line_service_host_test.c \
  app_program/subsystem3/line_barcode.c -o "$test_binary"
"$test_binary"
