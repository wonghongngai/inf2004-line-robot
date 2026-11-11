#!/bin/sh
set -eu
cd /c/Users/kangz/sit/embedded-sys-y2s1/project/inf2004-line-robot/build_make
export PATH=/c/Users/kangz/.pico-sdk/toolchain/15_2_Rel1/bin:/ucrt64/bin:/usr/bin:$PATH
export PICO_SDK_PATH=/c/Users/kangz/.pico-sdk/sdk/2.3.0
sources=$(find ../app_program -maxdepth 2 -name '*.c' ! -name '* *' -print | sort)
# Override the executable name at make invocation, including any profile
# cleanup, so the existing all_test ELF/map/UF2 files are never targeted.
output_name=${1:-barcode_slow_pwm20_trial}
make -rR -j8 "EXE_FILE=$output_name" ROBOT_MODE=all_test ALL_TEST_OFFLINE=0 TEST=0 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=cyw43 NET=lwip MQTT=1 E2U=tools/elf2uf2.exe "TEMP_SRCS=$sources" > build_barcode_slow_current.log 2>&1
