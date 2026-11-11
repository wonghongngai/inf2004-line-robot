"""Compile the production PWM functions against a simulated motor output."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / 'app_program/subsystem2/all_buddies_test.c').read_text()
header = (root / 'app_program/subsystem2/all_buddies_test.h').read_text()
macros = '\n'.join(line for line in (source + '\n' + header).splitlines()
                   if line.startswith('#define ALL_'))
functions = source[source.index('static void update_terrain_boost'):source.index('static void stop_test')]
white = source[source.index('static void white_follow_step'):source.index('static void report_motor_outputs')]
assert 't->left = 0.0f;' not in white and 't->right = 0.0f;' not in white
test = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#define LINE_BARCODE_PERIOD_MS 1u
#define tm_printf(...) ((void)0)
#define E_OK 0
enum {IMU_MOTION_NORMAL, IMU_MOTION_HUMP, IMU_MOTION_TILT, IMU_MOTION_COLLISION, IMU_MOTION_FAULT};
enum {IMU_EVENT_NONE, IMU_EVENT_HUMP, IMU_EVENT_TILT, IMU_EVENT_COLLISION, IMU_EVENT_FAULT};
typedef struct {
 float left, right, pwm_ramp_per_second;
 uint32_t pwm_updated;
 bool pwm_clock_started, barcode_slow, barcode_speed_reported;
 bool terrain_boost, terrain_reported;
 uint32_t terrain_last_ms, imu_updated;
 bool terrain_pitch_candidate;
 uint32_t terrain_pitch_since;
 int imu_er;
 struct { bool sensor_ok, calibrated; int motion_state, event, pitch_deg; } imu;
} ALL_TEST;
static float motor_left, motor_right;
static void drv_motor_set(float l, float r) { motor_left=l; motor_right=r; }
'''
test += macros + '\n' + functions + r'''
int main(void) {
 ALL_TEST t = {.left=10, .right=11, .pwm_ramp_per_second=ALL_TEST_PWM_RAMP_PER_SECOND,
               .pwm_clock_started=true};
 apply_follow_pwm(&t, 0, 11, 1);
 assert(motor_left > 9.95f && motor_left < 10 && motor_right == 11);
 for (unsigned i=2;i<=250;i++) apply_follow_pwm(&t,0,11,i);
 assert(motor_left < 0.001f);
 apply_follow_pwm(&t,10,10,251);
 assert(motor_left > 0 && motor_left < 0.05f);
 for (unsigned i=252;i<=500;i++) apply_follow_pwm(&t,10,10,i);
 assert(motor_left > 9.99f && motor_right == 11);
 t.barcode_slow=true;
 apply_follow_pwm(&t,10,10,501);
 assert(motor_left <= 9.501f && motor_right <= 10.451f);
 t.left=0; t.right=11; t.barcode_speed_reported=true;
 apply_follow_pwm(&t,10,10,502);
 assert(motor_left > 0 && motor_left < 0.05f && motor_right > 10.95f);
 apply_follow_pwm(&t,10,10,10000);
 assert(motor_left < 0.13f); /* delayed sample cannot jump to full target */
 t.imu.sensor_ok=true; t.imu.calibrated=true; t.imu_updated=10000;
 t.imu.motion_state=IMU_MOTION_HUMP;
 update_terrain_boost(&t,10000);
 assert(t.terrain_boost);
 t.pwm_ramp_per_second=0;
 apply_follow_pwm(&t,10,10,10001);
 assert(motor_left > 29.99f && motor_left < 30.01f &&
        motor_right > 32.99f && motor_right < 33.01f); /* boost overrides scan */
 assert(motor_right / motor_left > 1.099f && motor_right / motor_left < 1.101f);
 apply_follow_pwm(&t,0,11,10002);
 assert(motor_left == 0 && motor_right == 11); /* pivot not boosted */
 t.imu.motion_state=IMU_MOTION_NORMAL; t.imu_updated=10299;
 update_terrain_boost(&t,10299); assert(t.terrain_boost);
 t.imu_updated=10300; update_terrain_boost(&t,10300); assert(!t.terrain_boost);
 apply_follow_pwm(&t,10,10,10301);
 assert(motor_left < 9.501f && motor_right < 10.451f); /* scan resumes */
 t.barcode_slow=false; apply_follow_pwm(&t,10,10,10302);
 assert(motor_left == 10 && motor_right == 11);
 t.imu.motion_state=IMU_MOTION_TILT; update_terrain_boost(&t,10300);
 assert(t.terrain_boost);
 update_terrain_boost(&t,10401); assert(!t.terrain_boost); /* stale */
 t.imu_updated=10402; update_terrain_boost(&t,10402); assert(t.terrain_boost);
 t.imu.event=IMU_EVENT_COLLISION; update_terrain_boost(&t,10402);
 assert(!t.terrain_boost);
 t.imu.event=IMU_EVENT_NONE; t.imu.motion_state=IMU_MOTION_NORMAL;
 t.imu.pitch_deg=4; t.imu_updated=10800;
 update_terrain_boost(&t,10800); assert(!t.terrain_boost);
 t.imu_updated=10900; update_terrain_boost(&t,10900); assert(!t.terrain_boost);
 t.imu.pitch_deg=5; t.imu_updated=11000;
 update_terrain_boost(&t,11000); assert(!t.terrain_boost);
 t.imu_updated=11079; update_terrain_boost(&t,11079); assert(!t.terrain_boost);
 t.imu_updated=11080; update_terrain_boost(&t,11080); assert(t.terrain_boost);
 t.imu.pitch_deg=5; t.imu_updated=11500;
 update_terrain_boost(&t,11500); assert(t.terrain_boost);
 t.imu.pitch_deg=0; t.imu_updated=11799;
 update_terrain_boost(&t,11799); assert(t.terrain_boost);
 t.imu_updated=11800; update_terrain_boost(&t,11800); assert(!t.terrain_boost);
 t.imu.pitch_deg=-5; t.imu_updated=12000; update_terrain_boost(&t,12000);
 t.imu_updated=12080; update_terrain_boost(&t,12080); assert(t.terrain_boost);
 puts("PASS: steering ramp, terrain priority, settling, scan/cruise restoration, stale/collision cancellation");
}
'''
with tempfile.TemporaryDirectory() as temp:
    c = Path(temp) / 'ramp.c'
    exe = Path(temp) / 'ramp.exe'
    c.write_text(test)
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(c), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
