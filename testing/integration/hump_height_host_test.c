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
/* Kernel calls used by the S4 stuck-core guard (S4-U15). */
#define E_CTX (-25)
#define TSS_DDSP 0x01U
#define TSS_DINT 0x02U
#define TSS_INDP 0x04U
typedef struct { UINT sysstat; ID runtskid; ID schedtskid; } T_RSYS;
ER tk_ref_sys(T_RSYS *pk);
INT tk_get_prc(void);
ER tk_ena_dsp(void);
static UW fake_primask;
static UW get_primask(void) { return fake_primask; }
static void set_primask(UW pm) { fake_primask = pm; }

/* Exercise the actual IMU algorithm and snapshot code, replacing only I/O. */
#include "../../app_program/subsystem4/task_imu.c"
#include "../../app_program/subsystem2/wheel_odometry.h"

ID mtx_imu_data;
IMU_DATA s4_imu_data;
static unsigned prints, motor_stops, ena_dsp_calls;
static UINT fake_sysstat;
static ER dly_result;
static uint32_t fake_ms;
int tm_printf(UB *fmt, ...) { (void)fmt; prints++; return 0; }
void drv_motor_stop(void) { motor_stops++; }
ER tk_ref_sys(T_RSYS *pk)
{
    pk->sysstat = fake_sysstat | (fake_primask ? (TSS_DINT | TSS_DDSP) : 0U);
    pk->runtskid = pk->schedtskid = 1;
    return E_OK;
}
INT tk_get_prc(void) { return 2; }
ER tk_ena_dsp(void) { ena_dsp_calls++; fake_sysstat &= ~TSS_DDSP; return E_OK; }
ER tk_loc_mtx(ID id, TMO timeout) { (void)id; (void)timeout; return E_OK; }
ER tk_unl_mtx(ID id) { (void)id; return E_OK; }
ER tk_get_tim(SYSTIM *now) { now->hi = 0; now->lo = fake_ms; return E_OK; }
ER tk_dly_tsk(unsigned ms) { (void)ms; return dly_result; }
ER lsm303dlhc_init(void) { return E_OK; }
ER lsm303dlhc_read(LSM303_SAMPLE *sample) { (void)sample; return E_PAR; }
UW subsystem4_take_distance_mm(void) { return 0; }

/* 20 mm of raw rise (100 x 1 mm at 200 mg), after the S4 height
   calibration factor added in 7039135. */
#define RISE_20_MM ((W)((20000LL * S4_HEIGHT_CAL_NUM / S4_HEIGHT_CAL_DEN) / 1000))

#define BASE_Z 1000

static void settle_flat(void)
{
    reset_lift_detector();
    for (unsigned i = 0; i < 2u * S4_LIFT_FILTER_SAMPLES; ++i)
        assert(!update_lift(BASE_Z, BASE_Z, NULL));
}

/* Samples of a step to BASE_Z + step_mg until update_lift() first says
   lifted, worked out independently: the window average must reach the
   threshold, then hold it for the confirmation count. */
static unsigned expected_lift_samples(W step_mg)
{
    W mag = step_mg < 0 ? -step_mg : step_mg;
    unsigned k = 1;
    if (mag > S4_LIFT_CLIP_MG) mag = S4_LIFT_CLIP_MG;
    while ((mag * (W)k) / (W)S4_LIFT_FILTER_SAMPLES < S4_LIFT_DELTA_MG) ++k;
    return k + S4_LIFT_CONFIRM_SAMPLES - 1u;
}

static void step_detected(W step_mg)
{
    unsigned want = expected_lift_samples(step_mg);
    W delta = 0;
    settle_flat();
    for (unsigned i = 1; i < want; ++i)
        assert(!update_lift(BASE_Z + step_mg, BASE_Z, &delta));
    assert(update_lift(BASE_Z + step_mg, BASE_Z, &delta));
    assert((delta < 0 ? -delta : delta) >= S4_LIFT_DELTA_MG);
    /* Stays lifted while held, as the VC's latch expects. */
    for (unsigned i = 0; i < 20u; ++i) assert(update_lift(BASE_Z + step_mg, BASE_Z, NULL));
}

static void lift_checks(void)
{
    W delta = 0;

    /* Driving vibration: single samples swing +-500 mg (robot logs). */
    settle_flat();
    for (unsigned i = 0; i < 500u; ++i)
        assert(!update_lift(BASE_Z + ((i & 1u) ? 500 : -500), BASE_Z, &delta));
    /* Isolated spikes, as the old collision check tripped on (+-1300 mg). */
    settle_flat();
    for (unsigned i = 0; i < 500u; ++i)
        assert(!update_lift(BASE_Z + (i % 25u == 0u ? 1300 : 0), BASE_Z, NULL));
    /* Spikes in a burst (the logged "collisions" came in pairs): up to four
       same-sign 1300 mg samples in a row are clipped, so no lift. */
    for (unsigned burst = 1; burst <= 4u; ++burst) {
        settle_flat();
        for (unsigned i = 0; i < burst; ++i)
            assert(!update_lift(BASE_Z + 1300, BASE_Z, NULL));
        for (unsigned i = 0; i < 2u * S4_LIFT_FILTER_SAMPLES; ++i)
            assert(!update_lift(BASE_Z, BASE_Z, NULL));
    }

    /* Pick-up (Z up) and a tilted hold (Z down) are both lifts; a huge
       step is clipped but still detected. */
    step_detected(300);
    step_detected(-300);
    step_detected(2 * S4_LIFT_DELTA_MG);
    step_detected(1500);

    /* Just under the threshold, held for a long time: no lift. */
    settle_flat();
    for (unsigned i = 0; i < 500u; ++i)
        assert(!update_lift(BASE_Z + S4_LIFT_DELTA_MG - 1, BASE_Z, NULL));

    /* Warm-up: no decision until the window is full, even at a huge offset. */
    reset_lift_detector();
    for (unsigned i = 1; i < S4_LIFT_FILTER_SAMPLES + S4_LIFT_CONFIRM_SAMPLES - 1u; ++i)
        assert(!update_lift(BASE_Z + 800, BASE_Z, NULL));
    assert(update_lift(BASE_Z + 800, BASE_Z, NULL));

    /* Put down again: the condition clears and lift_reported re-arms. */
    lift_reported = TRUE;
    for (unsigned i = 0; i < 2u * S4_LIFT_FILTER_SAMPLES; ++i)
        (void)update_lift(BASE_Z, BASE_Z, NULL);
    assert(!update_lift(BASE_Z, BASE_Z, NULL) && !lift_reported && lift_confirm == 0u);
}

int main(void)
{
    reset_hump_detector(); reset_height_tracker();
    hump_state.first_sign = 1;
    for (unsigned i = 0; i < 100; ++i) update_height_estimate(1, 200);
    assert(current_height_mm == RISE_20_MM && peak_height_mm == (UW)RISE_20_MM && height_distance_seen);
    /* #5 (decided 20 Nov 2026): S4 reports the highest rise, so the descent
       must not reduce it. current_height_mm is internal and never published,
       so its value after the descent is not checked. */
    for (unsigned i = 0; i < 100; ++i) update_height_estimate(1, -200);
    assert(peak_height_mm == (UW)RISE_20_MM);
    reset_height_tracker(); hump_state.first_sign = -1;
    for (unsigned i = 0; i < 100; ++i) update_height_estimate(1, -200);
    assert(current_height_mm == RISE_20_MM && peak_height_mm == (UW)RISE_20_MM);
    update_height_estimate(10000, -500);
    assert(current_height_mm == S4_MAX_RAMP_HEIGHT_MM);
    update_height_estimate(10000, 500);
    assert(peak_height_mm == S4_MAX_RAMP_HEIGHT_MM);
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
    lift_checks();
    /* S4-U15 (21 Nov 2026 robot log): every failed IMU cycle printed a line,
       ~100/s once the core could not block. First failure prints at once,
       then at most one line per S4_ERROR_REPORT_MS. */
    prints = 0; fake_ms = 5000;
    for (unsigned i = 0; i < 2033; ++i) { report_error("init", E_CTX); fake_ms += 10; }
    /* 20.33 s of failures: first line + one per second. */
    assert(prints >= 20 && prints <= 22);
    assert(err_unreported < 2033);

    /* A healthy kernel: the retry delay works, motors are not touched. */
    prints = motor_stops = 0; dly_result = E_OK;
    s4_delay(S4_RETRY_DELAY_MS);
    assert(motor_stops == 0 && prints == 0);

    /* Core stuck with interrupts masked: tk_dly_tsk returns E_CTX. The guard
       stops the motors, prints one FATAL line and unmasks the core. */
    dly_result = E_CTX; fake_primask = 1U; fake_sysstat = 0U;
    s4_delay(S4_RETRY_DELAY_MS);
    assert(motor_stops == 1 && prints == 1 && fake_primask == 0U);
    assert(ena_dsp_calls == 0);
    /* Still stuck (dispatch disabled): motors stopped again, no new FATAL
       line, dispatching re-enabled. */
    fake_sysstat = TSS_DDSP;
    s4_delay(1);
    assert(motor_stops == 2 && prints == 1 && ena_dsp_calls == 1);
    /* Task-independent state cannot be repaired from a task: no tk_ena_dsp. */
    fake_sysstat = TSS_DDSP | TSS_INDP;
    s4_delay(1);
    assert(motor_stops == 3 && ena_dsp_calls == 1);
    dly_result = E_OK;
    puts("IMU fault path: rate-limited errors, stuck-core motor stop, unmask and re-enable dispatch PASS");

    puts("Hump height/odometry: fractional rise, descent, sign, bounds, missing calibration and validity PASS");
    puts("IMU lift (#53): vibration and single spikes ignored; pick-up and tilted hold detected after the filter and confirmation delay; window warm-up; release re-arms PASS");
    return 0;
}
