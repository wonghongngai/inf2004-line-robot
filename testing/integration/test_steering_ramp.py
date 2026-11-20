"""Compile the production PWM ramp from all_test against a simulated motor output.

Checks today's apply_follow_pwm()/approach_pwm(): the ramp rate comes from
ALL_TEST_PWM_RAMP_PER_SECOND, a delayed control cycle is capped at two
periods, ramp 0 applies targets at once, and the right-wheel straight trim is
kept. Expected values are derived from the firmware constants, not hard-coded.

The terrain-boost and barcode slow-down checks that used to live here were
removed (#3): that code was lost, and neither feature is a requirement.
"""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / 'app_program/subsystem2/all_buddies_test.c').read_text(encoding='utf-8')
header = (root / 'app_program/subsystem2/all_buddies_test.h').read_text(encoding='utf-8')
barcode_h = (root / 'app_program/subsystem3/line_barcode.h').read_text(encoding='utf-8')
period = re.search(r'#define LINE_BARCODE_PERIOD_MS\s+(\d+)u', barcode_h).group(1)
macros = '\n'.join(line for line in (source + '\n' + header).splitlines()
                   if line.startswith('#define ALL_'))
functions = source[source.index('static float approach_pwm'):source.index('static bool barcode_scanning')]
white_start = source.index('static void white_follow_step')
# Only this function: S5's obstacle code that follows it stops wheels on purpose.
white = source[white_start:source.index('\nstatic ', white_start + 1)]
# White-track steering must not cut a wheel to zero (it pivots too hard).
assert 't->left = 0.0f;' not in white and 't->right = 0.0f;' not in white

test = r'''
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#define LINE_BARCODE_PERIOD_MS ''' + period + r'''u
typedef struct {
    float left, right, pwm_ramp_per_second;
    uint32_t pwm_updated;
    bool pwm_clock_started;
} ALL_TEST;
static float motor_left, motor_right;
static void drv_motor_set(float l, float r) { motor_left = l; motor_right = r; }
''' + macros + '\n' + functions + r'''
#define NEAR(a, b) (fabsf((a) - (b)) < 1e-4f)
int main(void)
{
    const float step = ALL_TEST_PWM_RAMP_PER_SECOND * LINE_BARCODE_PERIOD_MS / 1000.0f;
    ALL_TEST t = {.left = 10, .right = 11,
                  .pwm_ramp_per_second = ALL_TEST_PWM_RAMP_PER_SECOND,
                  .pwm_clock_started = true};
    uint32_t now = 0;

    /* One period moves each wheel by exactly one ramp step, both directions. */
    now += LINE_BARCODE_PERIOD_MS;
    apply_follow_pwm(&t, 0, 11, now);
    assert(NEAR(motor_left, 10 - step) && NEAR(motor_right, 11));
    assert(motor_left == t.left && motor_right == t.right);

    /* Ramping down stops exactly at the target, never below it. */
    for (int i = 0; i < 100000 && t.left > 0; ++i) apply_follow_pwm(&t, 0, 11, now += LINE_BARCODE_PERIOD_MS);
    assert(motor_left == 0.0f);
    apply_follow_pwm(&t, 0, 11, now += LINE_BARCODE_PERIOD_MS);
    assert(motor_left == 0.0f);

    /* Ramping up starts with one step and stops exactly at the target. */
    apply_follow_pwm(&t, 10, 11, now += LINE_BARCODE_PERIOD_MS);
    assert(NEAR(motor_left, step));
    for (int i = 0; i < 100000 && t.left < 10; ++i) apply_follow_pwm(&t, 10, 11, now += LINE_BARCODE_PERIOD_MS);
    assert(motor_left == 10.0f && motor_right == 11.0f);

    /* A delayed cycle (here 10 s late) moves at most two periods' worth. */
    t.left = 0;
    apply_follow_pwm(&t, 10, 11, now += 10000u);
    assert(NEAR(motor_left, 2 * step));

    /* The first call after start-up counts as one period, not "since boot". */
    t.left = 0; t.pwm_clock_started = false;
    apply_follow_pwm(&t, 10, 11, 123456u);
    assert(NEAR(motor_left, step) && t.pwm_clock_started && t.pwm_updated == 123456u);

    /* Ramp 0 applies the targets immediately. */
    t.pwm_ramp_per_second = 0;
    apply_follow_pwm(&t, 3, 7, now += LINE_BARCODE_PERIOD_MS);
    assert(motor_left == 3.0f && motor_right == 7.0f);

    /* A straight request keeps the right-wheel trim. */
    apply_follow_pwm(&t, ALL_BASE_PWM, ALL_BASE_PWM, now += LINE_BARCODE_PERIOD_MS);
    assert(motor_left == ALL_BASE_PWM && motor_right == ALL_RIGHT_BASE_PWM);

    puts("PASS: ramp step from ALL_TEST_PWM_RAMP_PER_SECOND, exact stop at target, "
         "delayed-cycle cap, first-call period, ramp 0, straight trim");
    return 0;
}
'''
with tempfile.TemporaryDirectory() as temp:
    c = Path(temp) / 'ramp.c'
    exe = Path(temp) / 'ramp.exe'
    c.write_text(test)
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(c), '-o', str(exe), '-lm'],
                   check=True)
    subprocess.run([str(exe)], check=True)
