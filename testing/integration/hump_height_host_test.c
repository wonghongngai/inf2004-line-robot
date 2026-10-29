#include "robot_host_stubs.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>
#define LOCAL static
#define TMO_FEVR (-1)
typedef unsigned UINT;
typedef struct { uint32_t hi, lo; } SYSTIM;
ER tk_loc_mtx(ID id, TMO timeout);
ER tk_unl_mtx(ID id);
ER tk_get_tim(SYSTIM *now);
ER tk_dly_tsk(unsigned ms);

/* Exercise the actual IMU algorithm and snapshot code, replacing only I/O. */
#include "../../app_program/subsystem4/task_imu.c"
#include "../../app_program/subsystem2/wheel_odometry.h"

ID mtx_imu_data;
IMU_DATA s4_imu_data;
int tm_printf(UB *fmt, ...) { (void)fmt; return 0; }
ER tk_loc_mtx(ID id, TMO timeout) { (void)id; (void)timeout; return E_OK; }
ER tk_unl_mtx(ID id) { (void)id; return E_OK; }
ER tk_get_tim(SYSTIM *now) { now->lo = 0; return E_OK; }
ER tk_dly_tsk(unsigned ms) { (void)ms; return E_OK; }
ER lsm303dlhc_init(void) { return E_OK; }
ER lsm303dlhc_read(LSM303_SAMPLE *sample) { (void)sample; return E_PAR; }
UW subsystem4_take_distance_mm(void) { return 0; }

int main(void)
{
    reset_hump_detector(); reset_height_tracker();
    hump_state.first_sign = 1;
    for (unsigned i = 0; i < 100; ++i) update_height_estimate(1, 200);
    assert(current_height_mm == 20 && peak_height_mm == 20 && height_distance_seen);
    for (unsigned i = 0; i < 100; ++i) update_height_estimate(1, -200);
    assert(current_height_mm == 0 && peak_height_mm == 20);
    reset_height_tracker(); hump_state.first_sign = -1;
    for (unsigned i = 0; i < 100; ++i) update_height_estimate(1, -200);
    assert(current_height_mm == 20 && peak_height_mm == 20);
    update_height_estimate(10000, -500);
    assert(current_height_mm == S4_MAX_RAMP_HEIGHT_MM);
    update_height_estimate(10000, 500);
    assert(current_height_mm == 0 && peak_height_mm == S4_MAX_RAMP_HEIGHT_MM);
    reset_height_tracker(); update_height_estimate(0, 200);
    assert(!height_distance_seen && peak_height_mm == 0);

    WHEEL_ODOMETRY o;
    uint8_t mm;
    wheel_odometry_init(&o, 100.0f, 100.0f, 0, 0);
    assert(o.available);
    unsigned sum = 0;
    for (int i = 1; i <= 100; ++i) {
        assert(wheel_odometry_step(&o, i, -i, true, &mm));
        sum += mm;
    }
    /* No per-poll loss of fractions: 100 ticks at 100 ticks/cm is 10 mm.
     * Binary float may retain a tiny remainder until the next encoder edge. */
    assert(sum >= 9 && sum <= 10 && o.remainder_mm < 1.0f);
    wheel_odometry_init(&o, 10, 20, 0, 0);
    assert(wheel_odometry_step(&o, 10, -20, true, &mm) && mm == 10);
    assert(wheel_odometry_step(&o, 20, -40, false, &mm) && mm == 0);
    assert(wheel_odometry_step(&o, 30, -60, true, &mm) && mm == 10);
    wheel_odometry_init(&o, 0, 10, 0, 0);
    assert(!o.available && wheel_odometry_step(&o, 10, 10, true, &mm) && mm == 0);
    wheel_odometry_init(&o, NAN, 10, 0, 0); assert(!o.available);
    wheel_odometry_init(&o, INFINITY, 10, 0, 0); assert(!o.available);
    wheel_odometry_init(&o, 1, 1, 0, 0);
    assert(!wheel_odometry_step(&o, 100, 100, true, &mm));

    LSM303_SAMPLE sample = {0};
    publish_snapshot(&sample, 0, 0, 1000, 300, 10, 20, TRUE,
                     IMU_MOTION_HUMP, IMU_EVENT_HUMP, TRUE, TRUE, 1, FALSE);
    assert(s4_imu_data.ramp_height_valid && s4_imu_data.ramp_height_mm == 20);
    /* Startup ramp must not weaken the actual single-sample impact test. */
    LSM303_SAMPLE resting = {.az_mg = 1000};
    LSM303_SAMPLE vibration = {.ax_mg = 200, .ay_mg = 100, .az_mg = 900};
    LSM303_SAMPLE impact = {.ax_mg = S4_COLLISION_DELTA_MG, .az_mg = 1000};
    W delta;
    assert(!detect_collision(&vibration, &resting, &delta) && delta == 200);
    assert(detect_collision(&impact, &resting, &delta) && delta == S4_COLLISION_DELTA_MG);
    impact.ax_mg = -S4_COLLISION_DELTA_MG;
    assert(detect_collision(&impact, &resting, &delta) && delta == S4_COLLISION_DELTA_MG);
    puts("Hump height/odometry: fractional rise, descent, sign, bounds, missing calibration and validity PASS");
    puts("IMU collision: positive/negative single-sample impact threshold and measured delta PASS");
    return 0;
}
