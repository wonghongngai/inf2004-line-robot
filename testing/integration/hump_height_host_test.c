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
#define E_IO (-57)
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
/* S4-U17: fake 1 MHz hardware timer and a spin that advances it. */
static uint32_t fake_ms;
static unsigned spins, spun_ms;
#define S4_HW_NOW_US() ((UW)(fake_ms * 1000u))
#define S4_SPIN_MS(ms) (spins++, spun_ms += (ms), fake_ms += (ms))

/* Exercise the actual IMU algorithm and snapshot code, replacing only I/O. */
#include "../../app_program/subsystem4/task_imu.c"
#include "../../app_program/subsystem2/wheel_odometry.h"

ID mtx_imu_data;
IMU_DATA s4_imu_data;
static unsigned prints, motor_stops, ena_dsp_calls;
static UINT fake_sysstat;
static ER dly_result;
static unsigned bus_recover_calls;
static unsigned last_dly_ms;
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
ER tk_dly_tsk(unsigned ms) { last_dly_ms = ms; if (dly_result == E_OK) fake_ms += ms; return dly_result; }
BOOL lsm303dlhc_recover_bus(void) { bus_recover_calls++; return TRUE; }
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

/* #27 (23 Nov): hump detector rules tuned on the 10 mm one-side ramp.
   update_hump() takes the 10-sample average; values here stand for it. */
static void hump_fresh(void)
{
    hump_armed = TRUE; hump_rearm_count = 0U;
    reset_hump_detector(); reset_height_tracker();
}

static BOOL feed(unsigned count, W mg, BOOL moving, UW dist)
{
    W peak = 0;
    BOOL done = FALSE;
    for (unsigned i = 0; i < count; ++i)
        done = update_hump(mg, moving, dist, &peak) || done;
    return done;
}

static void hump_detector_checks(void)
{
    /* Bounds measured from the 23 Nov S4_TRACE logs (testing/subsystem4/logs/
       2026-11-23_hump_capture_S4.txt, _s4u09_ramp_passes_S4.txt, _s4u09_ramp_flat_S4.txt):
       on 16 s of flat driving the 10-sample average stayed >= 30 mg for at
       most 4 samples; ramp climbs held 40-90 mg for 59-89 samples (1.2-1.8 s). */
    assert(S4_HUMP_FILTER_SAMPLES >= 10u);
    assert(S4_HUMP_ENTRY_MG <= 40);
    assert(S4_HUMP_ENTRY_CONFIRM_SAMPLES > 4u);
    assert(S4_HUMP_EXIT_FLAT_SAMPLES >= 10u);
    assert(S4_HUMP_TIMEOUT_SAMPLES * S4_SAMPLE_PERIOD_MS >= 2000u);

    /* Entry needs the average at >= S4_HUMP_ENTRY_MG for the whole
       confirmation count: one sample short is not a hump. */
    hump_fresh();
    assert(!feed(S4_HUMP_ENTRY_CONFIRM_SAMPLES - 1u, 60, TRUE, 2));
    assert(hump_state.phase == HUMP_IDLE);
    feed(1, 0, TRUE, 2);
    assert(!feed(S4_HUMP_ENTRY_CONFIRM_SAMPLES - 1u, 60, TRUE, 2));
    assert(hump_state.phase == HUMP_IDLE);
    feed(1, 60, TRUE, 2);
    assert(hump_state.phase == HUMP_FIRST_SLOPE && hump_state.first_sign == 1);

    /* Only while the wheels turn: braking at a stop (no distance) never
       starts one, however long the jolt lasts. */
    hump_fresh();
    assert(!feed(50, 400, FALSE, 0));
    assert(hump_state.phase == HUMP_IDLE);

    /* The climb is counted from the last level sample, not from the
       confirmation: 20 samples at 25 mg (below entry) then the 10 that
       confirm at 60 mg, 2 mm each. */
    hump_fresh();
    feed(20, 25, TRUE, 2);
    feed(S4_HUMP_ENTRY_CONFIRM_SAMPLES, 60, TRUE, 2);
    assert(hump_state.phase == HUMP_FIRST_SLOPE);
    assert(hump_entry_height_milli_mm ==
           20 * 2 * 25 + (int64_t)S4_HUMP_ENTRY_CONFIRM_SAMPLES * 2 * 60);

    /* ... and restarts on level ground (|avg| <= S4_HUMP_SETTLE_MG) ... */
    hump_fresh();
    feed(20, 25, TRUE, 2);
    feed(1, S4_HUMP_SETTLE_MG, TRUE, 2);
    feed(S4_HUMP_ENTRY_CONFIRM_SAMPLES, 60, TRUE, 2);
    assert(hump_entry_height_milli_mm ==
           (int64_t)S4_HUMP_ENTRY_CONFIRM_SAMPLES * 2 * 60);

    /* ... or when the slope changes direction. */
    hump_fresh();
    feed(20, -25, TRUE, 2);
    feed(S4_HUMP_ENTRY_CONFIRM_SAMPLES, 60, TRUE, 2);
    assert(hump_entry_height_milli_mm ==
           (int64_t)S4_HUMP_ENTRY_CONFIRM_SAMPLES * 2 * 60);

    /* The end needs S4_HUMP_EXIT_FLAT_SAMPLES level samples in a row: a
       noise dip on the climb (the 6 mm pass, 23 Nov) does not end it. */
    hump_fresh();
    feed(S4_HUMP_ENTRY_CONFIRM_SAMPLES, 60, TRUE, 2);
    assert(!feed(S4_HUMP_EXIT_FLAT_SAMPLES - 1u, 15, TRUE, 2));
    assert(!feed(1, 50, TRUE, 2));
    assert(hump_state.phase == HUMP_FIRST_SLOPE);
    assert(!feed(S4_HUMP_EXIT_FLAT_SAMPLES - 1u, 15, TRUE, 2));
    assert(feed(1, 15, TRUE, 2));
    assert(hump_state.phase == HUMP_IDLE && !hump_armed);

    /* Not re-armed until S4_HUMP_REARM_FLAT_SAMPLES level samples. */
    assert(!feed(S4_HUMP_REARM_FLAT_SAMPLES - 1u, 0, TRUE, 2) && !hump_armed);
    feed(1, 0, TRUE, 2);
    assert(hump_armed);

    /* A climb that never levels off (robot stalled on the ramp) times out
       after S4_HUMP_TIMEOUT_SAMPLES with no event. */
    hump_fresh();
    feed(S4_HUMP_ENTRY_CONFIRM_SAMPLES, 60, TRUE, 2);
    assert(!feed(S4_HUMP_TIMEOUT_SAMPLES, 60, FALSE, 0));
    assert(hump_state.phase == HUMP_FIRST_SLOPE);
    assert(!feed(1, 60, FALSE, 0));
    assert(hump_state.phase == HUMP_IDLE && !hump_armed);
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
    s4_delay(S4_RETRY_MAX_MS);
    assert(motor_stops == 0 && prints == 0);

    /* Core stuck with interrupts masked: tk_dly_tsk returns E_CTX. The guard
       stops the motors, prints one FATAL line and unmasks the core. */
    dly_result = E_CTX; fake_primask = 1U; fake_sysstat = 0U;
    s4_delay(S4_RETRY_MAX_MS);
    assert(motor_stops == 1 && prints == 1 && fake_primask == 0U);
    /* Follow-up kernel report: not before 5 s, exactly once after. */
    wedge_followup(); assert(!wedge_followup_done);
    fake_ms += S4_WEDGE_FOLLOWUP_MS; wedge_followup(); assert(wedge_followup_done);
    wedge_followup(); assert(wedge_followup_done);
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

    /* S4-U17 (22 Nov 2026 robot log, 10 674 x er=-25). */
    /* Back-off doubles from one sample period and stops at the cap. */
    assert(s4_retry_delay_ms(1) == S4_SAMPLE_PERIOD_MS);
    assert(s4_retry_delay_ms(2) == 2u * S4_SAMPLE_PERIOD_MS);
    assert(s4_retry_delay_ms(3) == 4u * S4_SAMPLE_PERIOD_MS);
    assert(s4_retry_delay_ms(20) == S4_RETRY_MAX_MS);
    assert(s4_retry_delay_ms(1000000u) == S4_RETRY_MAX_MS);
    for (unsigned n = 1; n < 40; ++n)
        assert(s4_retry_delay_ms(n + 1) >= s4_retry_delay_ms(n));

    /* Bus fault (E_IO) for 60 s of retries: delays grow, the bus is freed
       every third failure, the data is marked invalid, and the console gets
       about one line per second, never one per retry. */
    fail_streak = 0; bus_recoveries = 0; bus_recover_calls = 0;
    err_reported = FALSE; err_unreported = 0; prints = 0;
    s4_imu_data.sensor_ok = TRUE; s4_imu_data.read_errors = 0;
    {
        uint32_t t0 = fake_ms;
        unsigned fails = 0;
        while (fake_ms - t0 < 60000u) {
            unsigned before = fake_ms;
            imu_failed("init", E_IO);
            ++fails;
            assert(last_dly_ms == s4_retry_delay_ms(fails));
            assert(fake_ms - before == last_dly_ms);
        }
        assert(fails < 80u);                       /* not thousands */
        assert(bus_recover_calls == fails / S4_RECOVER_EVERY_FAILS);
        assert(prints >= 55u && prints <= fails);  /* ~1 per second */
        assert(!s4_imu_data.sensor_ok && s4_imu_data.motion_state == IMU_MOTION_FAULT);
        assert(s4_imu_data.read_errors == fails);
    }
    /* A good read ends the streak (one INFO line) and the back-off restarts. */
    prints = 0;
    imu_read_ok();
    assert(fail_streak == 0u && prints == 1u);
    imu_read_ok();
    assert(prints == 1u);

    /* Kernel cannot block (E_CTX from tk_dly_tsk and from the I2C call, as
       on 22 Nov): no bus recovery (not a bus fault), and the loop still waits
       the back-off on the hardware timer, so 60 s of this is a few dozen
       retries and about 60 lines, not 10 000. */
    fail_streak = 0; bus_recover_calls = 0; prints = 0; spins = 0; spun_ms = 0;
    motor_stops = 0;
    dly_result = E_CTX;
    {
        uint32_t t0 = fake_ms;
        unsigned fails = 0;
        while (fake_ms - t0 < 60000u) { imu_failed("init", E_CTX); ++fails; }
        assert(fails < 80u);
        assert(bus_recover_calls == 0u);
        assert(spins == fails && spun_ms >= 60000u);
        assert(motor_stops == fails);
        assert(prints >= 55u && prints <= 65u);
    }
    dly_result = E_OK;
    puts("IMU dropout (S4-U17): exponential back-off, bus recovery every 3rd failure, ~1 error line/s, data invalid, spin-wait when the kernel cannot block PASS");

    hump_detector_checks();
    puts("Hump detector (#27): 200 ms entry confirmation, only while moving, climb counted from level ground, 200 ms level to end, re-arm, stall timeout PASS");
    puts("Hump height/odometry: fractional rise, descent, sign, bounds, missing calibration and validity PASS");
    puts("IMU lift (#53): vibration and single spikes ignored; pick-up and tilted hold detected after the filter and confirmation delay; window warm-up; release re-arms PASS");
    return 0;
}
