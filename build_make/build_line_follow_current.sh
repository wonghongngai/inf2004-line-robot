#!/bin/sh
set -eu
cd /c/Users/kangz/sit/embedded-sys-y2s1/project/inf2004-line-robot/build_make
export PATH=/c/Users/kangz/.pico-sdk/toolchain/15_2_Rel1/bin:/ucrt64/bin:/usr/bin:$PATH
export PICO_SDK_PATH=/c/Users/kangz/.pico-sdk/sdk/2.3.0
sources=$(find ../app_program -maxdepth 2 -name '*.c' ! -name '* *' -print | sort)
make -rR -j8 ROBOT_MODE=line_follow_test TEST=0 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=none NET=none MQTT=0 E2U=tools/elf2uf2.exe "TEMP_SRCS=$sources" > build_line_follow_current.log 2>&1
