#!/bin/sh
# u10_sizes.sh - S5-U10 (CR1): S5's data sizes before and after
# minimum-bit packing, as the RP2040 compiler lays them out, plus the RAM
# the image and S5's files use.  Needs a fresh `sh build.sh integrated`
# (reads build_make/build_integrated.log for the exact compiler flags).
#
# Usage, from the repo root:
#   sh testing/subsystem5/u10_sizes.sh | tee testing/subsystem5/logs/<date>_S5-U10_sizes.txt
set -eu

ROOT=$(cd "$(dirname "$0")/../.." && pwd)
cd "$ROOT/build_make"
LOG=build_integrated.log
[ -f "$LOG" ] || { echo "run: sh build.sh integrated" >&2; exit 2; }

WORK=$(mktemp -d 2>/dev/null || mktemp -d -t u10)
trap 'rm -rf "$WORK"' EXIT INT TERM

# "Before": the unpacked structs, one constant of each, sizes read with nm.
cat > "$WORK/u10_sizes.c" <<'EOF'
#include "subsystem5.h"
#include "obstacle_logic.h"
const S5_REQUEST         u10_request_struct  = {0};
const S5_OBSTACLE_INFO   u10_obstacle_struct = {0};
const UW                 u10_packed_word     = 0U;
EOF

# The flags task_obstacle.c was built with, minus its output/dependency parts.
FLAGS=$(grep -m1 -o 'arm-none-eabi-gcc -mcpu.*-c -o "mtkernel_3/app_program/subsystem5/task_obstacle.o"' "$LOG" |
        sed 's/-c -o .*//; s/-MMD -MP//; s/-MF"[^"]*"//; s/-MT"[^"]*"//')
[ -n "$FLAGS" ] || { echo "task_obstacle.o compile line not found in $LOG" >&2; exit 2; }
SRC=$WORK/u10_sizes.c
OBJ=$WORK/u10_sizes.o
if command -v cygpath >/dev/null 2>&1; then SRC=$(cygpath -m "$SRC"); OBJ=$(cygpath -m "$OBJ"); fi
eval "$FLAGS -I../app_program/subsystem5 -c \"$SRC\" -o \"$OBJ\""

echo "S5-U10 sizes  ($(git rev-parse --short HEAD)$(git diff --quiet HEAD || echo ' + uncommitted changes'))"
echo
echo "== Shared S5 data: unpacked struct (before) vs packed word (after), bytes"
arm-none-eabi-nm -S --size-sort "$OBJ" | grep ' u10_' | while read -r _addr size _type name; do
    printf '  %-22s %d\n' "$name" "$((0x$size))"
done
echo
echo "== The packed words in the image (robot-integrated.elf)"
arm-none-eabi-nm -S robot-integrated.elf | grep -E ' (g_s5_request|g_s5_obstacle)$' || true
echo
echo "== Image (arm-none-eabi-size): RAM = data + bss"
arm-none-eabi-size robot-integrated.elf
echo
echo "== Subsystem 5 objects"
arm-none-eabi-size mtkernel_3/app_program/subsystem5/*.o
