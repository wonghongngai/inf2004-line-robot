/*
 * task_imu.c
 * Buddy 4 -- IMU & Terrain Monitoring (final requirements).
 *
 * Ramp-height method:
 *   - Buddy 4 detects the ramp from the IMU.
 *   - The forward-axis gravity component gives sin(ramp angle):
 *         sin(theta) ~= forward_mg / 1000
 *   - Buddy 2 supplies incremental wheel distance d.
 *   - Buddy 4 calculates vertical rise:
 *         delta_height ~= d * sin(theta)
 *                      ~= d * forward_mg / 1000
 *   - The maximum accumulated rise is reported as ramp height.
 */

#include "subsystem4.h"
#include "drv_lsm303dlhc.h"
#include <stdint.h>
/* Sample counts at S4_SAMPLE_PERIOD_MS = 20 ms (halved 20 Nov from the 10 ms values). */
#define S4_HUMP_REARM_FLAT_SAMPLES 25U     /* 500 ms */
#define S4_HUMP_ENTRY_CONFIRM_SAMPLES 3U  /* 60 ms (5 at 10 ms) */
#define S4_MOVEMENT_HOLD_SAMPLES 5U       /* 100 ms */
#define S4_HUMP_FILTER_SAMPLES 3U         /* 60 ms moving average (5 at 10 ms) */
#define S4_HUMP_EXIT_FLAT_MG 40
#define S4_HUMP_EXIT_FLAT_SAMPLES 4U      /* 80 ms */
#define S4_HEIGHT_CAL_NUM 25
#define S4_HEIGHT_CAL_DEN 11
typedef enum {
    HUMP_IDLE = 0,
    HUMP_FIRST_SLOPE,
    HUMP_OPPOSITE_SLOPE,
    HUMP_SETTLING
} HUMP_PHASE;

typedef struct {
    W sum_x;
    W sum_y;
    W sum_z;
    UW count;
} CAL_STATE;

typedef struct {
    HUMP_PHASE phase;
    INT first_sign;
    BOOL descent_started;
    W peak_mg;
    UW age_samples;
    UW settle_samples;

    /* Entry confirmation prevents short vibrations from becoming humps. */
    INT entry_sign;
    UW entry_samples;
} HUMP_STATE;

LOCAL CAL_STATE cal_state;
LOCAL HUMP_STATE hump_state;
LOCAL UW tilt_samples;
LOCAL BOOL tilt_reported;
LOCAL BOOL hump_armed;
LOCAL UW hump_rearm_count;
LOCAL UW movement_hold_samples;
LOCAL int64_t hump_entry_height_milli_mm;

/* Lift check (#53): moving average of the clipped Z deviation and the
 * consecutive-sample count. */
LOCAL W lift_samples[S4_LIFT_FILTER_SAMPLES];
LOCAL W lift_sum;
LOCAL UW lift_next;
LOCAL UW lift_count;
LOCAL UW lift_confirm;
LOCAL BOOL lift_reported;

/* Height estimator state for one ramp crossing. */
LOCAL W current_height_mm;
LOCAL UW peak_height_mm;
/* Retain fractional mm: mm * mg / 1000 loses small rises if each sample
 * is rounded separately. 1000 units here represent one millimetre. */
LOCAL int64_t current_height_milli_mm;
LOCAL BOOL height_distance_seen;

LOCAL W abs_w(W value)
{
    return (value < 0) ? -value : value;
}

LOCAL INT sign_w(W value)
{
    return (value >= 0) ? 1 : -1;
}

LOCAL void reset_hump_detector(void)
{
    hump_state.phase = HUMP_IDLE;
    hump_state.first_sign = 0;
    hump_state.descent_started = FALSE;
    hump_state.peak_mg = 0;
    hump_state.age_samples = 0U;
    hump_state.settle_samples = 0U;

    hump_state.entry_sign = 0;
    hump_state.entry_samples = 0U;
    hump_entry_height_milli_mm = 0;
}

LOCAL void reset_height_tracker(void)
{
    current_height_mm = 0;
    peak_height_mm = 0U;
    current_height_milli_mm = 0;
    height_distance_seen = FALSE;
}

/*
 * Estimate pitch for logging/telemetry without floating-point trigonometry.
 * Uses the IMU Y axis as the forward/ramp axis for the current mounting.
 * The table maps |forward-baseline| in mg to the nearest 5-degree pitch angle.
 */
LOCAL W estimate_pitch_deg(W forward_mg)
{
    static const W sin_table_mg[] = {
        0, 87, 174, 259, 342, 423, 500, 574, 643, 707,
        766, 819, 866, 906, 940, 966, 985, 996, 1000
    };
    W value;
    INT sign;
    UW i;

    sign = sign_w(forward_mg);
    value = abs_w(forward_mg);

    if (value >= 1000) {
        return (W)(90 * sign);
    }

    for (i = 1U; i < 19U; i++) {
        if (value < sin_table_mg[i]) {
            W lower = sin_table_mg[i - 1U];
            W upper = sin_table_mg[i];

            if ((value - lower) < (upper - value)) {
                return (W)((INT)(i - 1U) * 5 * sign);
            }
            return (W)((INT)i * 5 * sign);
        }
    }

    return (W)(90 * sign);
}

LOCAL W calibrated_height_mm(int64_t raw_milli_mm)
{
    int64_t calibrated_milli_mm;

    if (raw_milli_mm <= 0) {
        return 0;
    }

    calibrated_milli_mm =
        (raw_milli_mm * S4_HEIGHT_CAL_NUM) /
        S4_HEIGHT_CAL_DEN;

    /* Cap after calibration: telemetry carries 0-255 mm (8 bits). */
    if (calibrated_milli_mm > (int64_t)S4_MAX_RAMP_HEIGHT_MM * 1000) {
        calibrated_milli_mm = (int64_t)S4_MAX_RAMP_HEIGHT_MM * 1000;
    }

    return (W)(calibrated_milli_mm / 1000);
}

LOCAL void update_height_estimate(UW distance_mm, W forward_mg)
{
    W aligned_forward;

    if (distance_mm == 0U || hump_state.first_sign == 0) {
        return;
    }

    /*
     * first_sign is the uphill direction.
     * On the current mounting, nose-up is negative Y, so a negative
     * forward value with first_sign == -1 becomes a positive rise.
     */
    aligned_forward = (W)(forward_mg * hump_state.first_sign);

    if (aligned_forward > 1000) {
        aligned_forward = 1000;
    }

    /*
     * We report maximum vertical rise. Once the robot is travelling
     * down the opposite side, do not subtract that descent from the
     * height already accumulated on the climb.
     */
    if (aligned_forward <= 0) {
        return;
    }

    height_distance_seen = TRUE;

    /*
     * delta_height ~= distance_mm * sin(theta)
     *              ~= distance_mm * aligned_forward / 1000
     *
     * Keep milli-mm precision. 1000 stored units = 1 mm.
     */
    current_height_milli_mm +=
        (int64_t)distance_mm * (int64_t)aligned_forward;

    if (current_height_milli_mm >
        (int64_t)S4_MAX_RAMP_HEIGHT_MM * 1000) {

        current_height_milli_mm =
            (int64_t)S4_MAX_RAMP_HEIGHT_MM * 1000;
    }

    current_height_mm =
        calibrated_height_mm(current_height_milli_mm);

    if ((UW)current_height_mm > peak_height_mm) {
        peak_height_mm = (UW)current_height_mm;
    }
}
LOCAL void publish_snapshot(const LSM303_SAMPLE *sample,
                            W baseline_x,
                            W baseline_y,
                            W baseline_z,
                            W hump_peak,
                            W pitch_deg,
                            UW ramp_height_mm,
                            BOOL ramp_height_valid,
                            IMU_MOTION_STATE motion,
                            IMU_EVENT event,
                            BOOL calibrated,
                            BOOL sensor_ok,
                            UW cycle_ms,
                            BOOL timing_overrun)
{
    ER er;

    er = tk_loc_mtx(mtx_imu_data, TMO_FEVR);
    if (er < E_OK) {
        return;
    }

    if (sample != NULL) {
        s4_imu_data.ax_mg = sample->ax_mg;
        s4_imu_data.ay_mg = sample->ay_mg;
        s4_imu_data.az_mg = sample->az_mg;
        s4_imu_data.mx_raw = sample->mx_raw;
        s4_imu_data.my_raw = sample->my_raw;
        s4_imu_data.mz_raw = sample->mz_raw;
    }

    s4_imu_data.baseline_ax_mg = baseline_x;
    s4_imu_data.baseline_ay_mg = baseline_y;
    s4_imu_data.baseline_az_mg = baseline_z;
    s4_imu_data.hump_peak_mg = hump_peak;
    s4_imu_data.pitch_deg = pitch_deg;
    s4_imu_data.ramp_height_mm = ramp_height_mm;
    s4_imu_data.ramp_height_valid = ramp_height_valid;
    s4_imu_data.motion_state = motion;
    s4_imu_data.event = event;
    s4_imu_data.calibrated = calibrated;
    s4_imu_data.sensor_ok = sensor_ok;
    s4_imu_data.last_cycle_ms = cycle_ms;
    if (cycle_ms > s4_imu_data.max_cycle_ms) {
        s4_imu_data.max_cycle_ms = cycle_ms;
    }

    if (timing_overrun) {
        s4_imu_data.timing_overruns++;
    }

    if (sample != NULL && sensor_ok) {
        s4_imu_data.sample_count++;
    }

    tk_unl_mtx(mtx_imu_data);
}

/*
 * IMU fault handling (robot log 21 Nov 2026, 22:55 boot; also 20 Nov ramp
 * boot 3). After a fault, every kernel wait made by this task, tk_dly_tsk()
 * included, returned E_CTX at once: this task's core could no longer block or
 * dispatch. The 200 ms retry became a busy loop printing ~100 lines/s, and
 * motion_task (pinned to core 1, priority 14) never ran again, so the VC could
 * neither halt the robot nor handle PAUSE. Only this task was still running,
 * so this task has to stop the motors and try to recover the core.
 */
#define S4_RETRY_DELAY_MS       200
#define S4_ERROR_REPORT_MS      1000U
#define S4_ERROR_MTX_TMO_MS     20

/* Weak: links in modes that build S4 without the S2 motor driver. */
extern void drv_motor_stop(void) __attribute__((weak));

LOCAL UW err_unreported;
LOCAL UW err_last_report_ms;
LOCAL BOOL err_reported;
LOCAL BOOL wedge_reported;

LOCAL UW s4_now_ms(void)
{
    SYSTIM t;

    tk_get_tim(&t);
    return t.lo;
}

/* Print the first failure at once, then at most one line per second. */
LOCAL void report_error(const char *what, ER er)
{
    UW now = s4_now_ms();

    err_unreported++;
    if (err_reported &&
        (UW)(now - err_last_report_ms) < S4_ERROR_REPORT_MS) {
        return;
    }
    LOG_ERR(TAG_S4_IMU, "IMU %s failed, er=%d (%u failures since last report)",
            what, er, err_unreported);
    err_unreported = 0U;
    err_last_report_ms = now;
    err_reported = TRUE;
}

/* The kernel refused to block this task: its core is wedged. */
LOCAL void handle_unblockable_core(ER er)
{
    T_RSYS rsys;
    UW primask = get_primask();
    UINT sysstat = (tk_ref_sys(&rsys) >= E_OK) ? rsys.sysstat : 0xFFU;

    if (drv_motor_stop != NULL) {
        drv_motor_stop();   /* register writes only: works with no kernel */
    }
    if (!wedge_reported) {
        wedge_reported = TRUE;
        LOG_ERR(TAG_S4_IMU, "FATAL core %d cannot block (er=%d sysstat=0x%x primask=%u run=%d sched=%d); motors stopped",
                tk_get_prc(), er, sysstat, primask,
                (INT)rsys.runtskid, (INT)rsys.schedtskid);
    }
    /* This task holds no lock here, so a set PRIMASK was leaked by someone
       else. Unmask so the tick and PendSV run and the VC can latch its stop. */
    if (primask != 0U) {
        set_primask(0U);
    }
    if (tk_ref_sys(&rsys) >= E_OK && (rsys.sysstat & TSS_DDSP) != 0U &&
        (rsys.sysstat & (TSS_DINT | TSS_INDP)) == 0U) {
        (void)tk_ena_dsp();
    }
}

LOCAL void s4_delay(TMO ms)
{
    ER er = tk_dly_tsk(ms);

    if (er < E_OK) {
        handle_unblockable_core(er);
    }
}

LOCAL void record_read_error(void)
{
    if (tk_loc_mtx(mtx_imu_data, S4_ERROR_MTX_TMO_MS) >= E_OK) {
        s4_imu_data.read_errors++;
        s4_imu_data.sensor_ok = FALSE;
        s4_imu_data.motion_state = IMU_MOTION_FAULT;
        s4_imu_data.event = IMU_EVENT_FAULT;
        tk_unl_mtx(mtx_imu_data);
    }
}

LOCAL BOOL update_calibration(const LSM303_SAMPLE *sample,
                              W *baseline_x,
                              W *baseline_y,
                              W *baseline_z)
{
    cal_state.sum_x += sample->ax_mg;
    cal_state.sum_y += sample->ay_mg;
    cal_state.sum_z += sample->az_mg;
    cal_state.count++;

    if (cal_state.count < S4_CALIBRATION_SAMPLES) {
        return FALSE;
    }

    *baseline_x = cal_state.sum_x / (W)cal_state.count;
    *baseline_y = cal_state.sum_y / (W)cal_state.count;
    *baseline_z = cal_state.sum_z / (W)cal_state.count;

    LOG_INFO(TAG_S4_IMU, "calibration complete base=(%d,%d,%d) mg",
              *baseline_x, *baseline_y, *baseline_z);

    return TRUE;
}

LOCAL void reset_lift_detector(void)
{
    UW i;

    for (i = 0U; i < S4_LIFT_FILTER_SAMPLES; i++) {
        lift_samples[i] = 0;
    }
    lift_sum = 0;
    lift_next = 0U;
    lift_count = 0U;
    lift_confirm = 0U;
    lift_reported = FALSE;
}

/*
 * Lift (#53) = the averaged vertical acceleration stays away from the
 * calibrated baseline. Picking the robot up accelerates it upwards (Z rises)
 * and holding it tilted lowers Z; driving vibration averages out. Returns
 * TRUE on every sample while the condition has held for
 * S4_LIFT_CONFIRM_SAMPLES; *filtered_delta_mg is the averaged (clipped)
 * Z deviation from the baseline.
 */
LOCAL BOOL update_lift(W az_mg, W baseline_az_mg, W *filtered_delta_mg)
{
    W delta;
    W deviation = az_mg - baseline_az_mg;

    if (deviation > S4_LIFT_CLIP_MG) {
        deviation = S4_LIFT_CLIP_MG;
    } else if (deviation < -S4_LIFT_CLIP_MG) {
        deviation = -S4_LIFT_CLIP_MG;
    }

    if (lift_count == S4_LIFT_FILTER_SAMPLES) {
        lift_sum -= lift_samples[lift_next];
    } else {
        lift_count++;
    }
    lift_samples[lift_next] = deviation;
    lift_sum += deviation;
    lift_next = (lift_next + 1U) % S4_LIFT_FILTER_SAMPLES;

    delta = lift_sum / (W)lift_count;
    if (filtered_delta_mg != NULL) {
        *filtered_delta_mg = delta;
    }

    /* Only a full window is averaged enough to judge. */
    if (lift_count == S4_LIFT_FILTER_SAMPLES &&
        abs_w(delta) >= S4_LIFT_DELTA_MG) {
        if (lift_confirm < S4_LIFT_CONFIRM_SAMPLES) {
            lift_confirm++;
        }
    } else {
        lift_confirm = 0U;
        lift_reported = FALSE;
    }

    return (lift_confirm >= S4_LIFT_CONFIRM_SAMPLES) ? TRUE : FALSE;
}

LOCAL BOOL update_tilt(W lateral_mg, W forward_mg, BOOL hump_active)
{
    BOOL beyond_threshold;

    /*
     * X = lateral tilt.
     * A large sideways tilt remains dangerous even while crossing a ramp.
     *
     * Y = forward pitch.
     * A large Y deviation is expected while a ramp/hump is active,
     * so only treat forward pitch as tilt when not traversing a hump.
     */
    beyond_threshold =
        (abs_w(lateral_mg) >= S4_TILT_DELTA_MG) ||
        (!hump_active &&
         abs_w(forward_mg) >= S4_TILT_DELTA_MG);

    if (beyond_threshold) {

        if (tilt_samples < S4_TILT_CONFIRM_SAMPLES) {
            tilt_samples++;
        }

    } else {

        tilt_samples = 0U;
        tilt_reported = FALSE;
    }

    return
        (tilt_samples >= S4_TILT_CONFIRM_SAMPLES)
        ? TRUE
        : FALSE;
}

/* Multi-sample ramp pattern: up slope -> opposite slope -> flat again. */
LOCAL BOOL update_hump(W forward_mg,
                       BOOL moving,
                       UW distance_mm,
                       W *completed_peak)
{
    W abs_forward;

    abs_forward = abs_w(forward_mg);

    if (completed_peak != NULL) {
        *completed_peak = 0;
    }

    /*
     * After completion or timeout, require a sustained flat period
     * before another slope can start a measurement.
     */
    if (!hump_armed) {

        if (abs_forward <= S4_HUMP_SETTLE_MG) {

            if (hump_rearm_count < S4_HUMP_REARM_FLAT_SAMPLES) {
                hump_rearm_count++;
            }

            if (hump_rearm_count >= S4_HUMP_REARM_FLAT_SAMPLES) {
                hump_armed = TRUE;
                hump_rearm_count = 0U;

                LOG_INFO(TAG_S4_IMU, "hump detector re-armed");
            }

        } else {
            hump_rearm_count = 0U;
        }

        return FALSE;
    }

    if (hump_state.phase != HUMP_IDLE) {
        hump_state.age_samples++;

        if (hump_state.age_samples > S4_HUMP_TIMEOUT_SAMPLES) {
            hump_armed = FALSE;
            hump_rearm_count = 0U;
            reset_hump_detector();
            reset_height_tracker();
            LOG_WARN(TAG_S4_IMU, "hump timed out; waiting for level ground");
            return FALSE;
        }
    }

    switch (hump_state.phase) {

    case HUMP_IDLE:
    {
        INT current_sign;

        /*
         * A hump may only begin when:
         *   1. the robot has moved recently, and
         *   2. the filtered forward-axis tilt reaches the entry threshold.
         *
         * Consecutive same-direction samples are required so a
         * short motor-start vibration does not become a hump.
         */
        if (moving && abs_forward >= S4_HUMP_ENTRY_MG) {

            current_sign = sign_w(forward_mg);

            if (hump_state.entry_samples == 0U) {
                hump_state.entry_sign = current_sign;
                hump_state.entry_samples = 1U;
                hump_state.peak_mg = abs_forward;

                hump_entry_height_milli_mm =
                    (int64_t)distance_mm * (int64_t)abs_forward;

            } else if (current_sign == hump_state.entry_sign) {
                hump_state.entry_samples++;

                hump_entry_height_milli_mm +=
                    (int64_t)distance_mm * (int64_t)abs_forward;

                if (abs_forward > hump_state.peak_mg) {
                    hump_state.peak_mg = abs_forward;
                }

            } else {
                /*
                 * Direction changed before confirmation, so restart the
                 * candidate sequence using this new direction.
                 */
                hump_state.entry_sign = current_sign;
                hump_state.entry_samples = 1U;
                hump_state.peak_mg = abs_forward;

                hump_entry_height_milli_mm =
                    (int64_t)distance_mm * (int64_t)abs_forward;
            }

            if (hump_state.entry_samples >=
                S4_HUMP_ENTRY_CONFIRM_SAMPLES) {

                hump_state.phase = HUMP_FIRST_SLOPE;
                hump_state.first_sign = hump_state.entry_sign;
                hump_state.age_samples = 0U;
                hump_state.settle_samples = 0U;

                hump_state.entry_sign = 0;
                hump_state.entry_samples = 0U;

                LOG_INFO(TAG_S4_IMU,
                          "hump entry confirmed sign=%d peak=%dmg",
                          hump_state.first_sign,
                          hump_state.peak_mg);
            }

        } else {
            hump_state.entry_sign = 0;
            hump_state.entry_samples = 0U;
            hump_state.peak_mg = 0;
            hump_entry_height_milli_mm = 0;
        }

        break;
    }

    case HUMP_FIRST_SLOPE:

        LOG_INFO(TAG_S4_IMU,
            "HUMP ACTIVE: forward=%d abs=%d dist=%u height=%u",
            forward_mg,
            abs_forward,
            distance_mm,
            peak_height_mm);

        if (abs_forward > hump_state.peak_mg) {
            hump_state.peak_mg = abs_forward;
        }

        /*
        * Once a confirmed hump returns close to level ground,
        * begin checking that it remains flat.
        */
        if (abs_forward <= S4_HUMP_EXIT_FLAT_MG) {
            hump_state.phase = HUMP_SETTLING;
            hump_state.settle_samples = 1U;
        }

        break;

    case HUMP_OPPOSITE_SLOPE:

        if (abs_forward > hump_state.peak_mg) {
            hump_state.peak_mg = abs_forward;
        }

        /*
         * Moving measurements are noisy, so use a slightly wider
         * "approximately flat" threshold for completion.
         */
        if (abs_forward <= S4_HUMP_EXIT_FLAT_MG) {
            hump_state.phase = HUMP_SETTLING;
            hump_state.settle_samples = 1U;
        }

        break;

    case HUMP_SETTLING:

        if (abs_forward <= S4_HUMP_EXIT_FLAT_MG) {

            hump_state.settle_samples++;

            if (hump_state.settle_samples >=
                S4_HUMP_EXIT_FLAT_SAMPLES) {

                if (completed_peak != NULL) {
                    *completed_peak = hump_state.peak_mg;
                }

                hump_armed = FALSE;
                hump_rearm_count = 0U;

                reset_hump_detector();

                return TRUE;
            }

        } else {

            /*
            * Robot is not flat yet.
            * Continue tracking the hump.
            */
            hump_state.settle_samples = 0U;
            hump_state.phase = HUMP_FIRST_SLOPE;
        }

        break;
    default:
        reset_hump_detector();
        reset_height_tracker();
        break;
    }

    return FALSE;
}

LOCAL void print_periodic_status(const LSM303_SAMPLE *sample,
                                  W baseline_x,
                                  W baseline_y,
                                  IMU_MOTION_STATE motion,
                                  W pitch_deg,
                                  UW ramp_height_mm,
                                  BOOL ramp_height_valid,
                                  UW sample_count)
{
    W dx;
    W dy;

    if ((sample_count % S4_PRINT_PERIOD_SAMPLES) != 0U) {
        return;
    }

    dx = sample->ax_mg - baseline_x;
    dy = sample->ay_mg - baseline_y;

    LOG_INFO(TAG_S4_IMU,
        "n=%u "
        "acc=(%d,%d,%d)mg "
        "delta=(x:%d,y:%d)mg "
        "mag=(%d,%d,%d) "
        "pitch=%d "
        "height=%umm "
        "height_valid=%d "
        "state=%d",

        sample_count,

        sample->ax_mg,
        sample->ay_mg,
        sample->az_mg,

        dx,
        dy,

        sample->mx_raw,
        sample->my_raw,
        sample->mz_raw,

        pitch_deg,
        ramp_height_mm,
        ramp_height_valid,
        motion
    );
}

EXPORT void imu_task(INT stacd, void *exinf)
{
    LSM303_SAMPLE sample;
    SYSTIM cycle_start;
    SYSTIM cycle_end;
    W baseline_x = 0;
    W baseline_y = 0;
    W baseline_z = 0;
    W dx;
    W dy;
    W completed_hump_peak;
    /* Moving average of the last five forward samples for hump detection. */
    W hump_samples[S4_HUMP_FILTER_SAMPLES] = {0};
    W hump_sum = 0;
    UW hump_next = 0U;
    UW hump_count = 0U;
    W last_hump_peak = 0;
    W pitch_deg = 0;
    W lift_delta_mg = 0;
    UW last_ramp_height = 0U;
    BOOL last_ramp_height_valid = FALSE;
    UW distance_mm;
    UW cycle_ms;
    UW local_sample_count = 0U;
    BOOL calibrated = FALSE;
    BOOL sensor_ok = FALSE;
    BOOL tilt_now;
    BOOL hump_was_active;
    BOOL hump_is_active;
    BOOL hump_completed;
    BOOL timing_overrun;
    IMU_MOTION_STATE motion;
    IMU_EVENT event;
    ER er;

    (void)stacd;
    (void)exinf;

    cal_state.sum_x = 0;
    cal_state.sum_y = 0;
    cal_state.sum_z = 0;
    cal_state.count = 0U;
    tilt_samples = 0U;
    tilt_reported = FALSE;
    hump_armed = TRUE;
    hump_rearm_count = 0U;
    movement_hold_samples = 0U;
    reset_hump_detector();
    reset_height_tracker();
    reset_lift_detector();

    LOG_INFO(TAG_S4_IMU, "IMU task started, period=%d ms",
              S4_SAMPLE_PERIOD_MS);

    while (1) {
        tk_get_tim(&cycle_start);
        timing_overrun = FALSE;

        if (!sensor_ok) {
            er = lsm303dlhc_init();
            if (er < E_OK) {
                record_read_error();
                report_error("init", er);
                s4_delay(S4_RETRY_DELAY_MS);
                continue;
            }

            sensor_ok = TRUE;
            err_reported = FALSE;   /* print the next first failure at once */
            err_unreported = 0U;
            calibrated = FALSE;
            cal_state.sum_x = 0;
            cal_state.sum_y = 0;
            cal_state.sum_z = 0;
            cal_state.count = 0U;
            tilt_samples = 0U;
            tilt_reported = FALSE;
            hump_armed = TRUE;
            hump_rearm_count = 0U;
            movement_hold_samples = 0U;
            hump_sum = 0;
            hump_next = 0U;
            hump_count = 0U;
            reset_hump_detector();
            reset_height_tracker();
            reset_lift_detector();
        }

        er = lsm303dlhc_read(&sample);
        if (er < E_OK) {
            sensor_ok = FALSE;
            record_read_error();
            report_error("read", er);
            s4_delay(S4_RETRY_DELAY_MS);
            continue;
        }

        local_sample_count++;
        motion = IMU_MOTION_NORMAL;
        event = IMU_EVENT_NONE;

        if (!calibrated) {
            calibrated = update_calibration(&sample,
                                            &baseline_x,
                                            &baseline_y,
                                            &baseline_z);

            publish_snapshot(&sample,
                             baseline_x, baseline_y, baseline_z,
                             last_hump_peak,
                             0,
                             last_ramp_height,
                             last_ramp_height_valid,
                             IMU_MOTION_NORMAL,
                             IMU_EVENT_NONE,
                             calibrated,
                             TRUE,
                             0U,
                             FALSE);
        } else {
            dx = sample.ax_mg - baseline_x;
            dy = sample.ay_mg - baseline_y;

            /*
            * Physical mounting confirmed from ramp testing:
            *
            * X = lateral / sideways movement
            * Y = forward / ramp slope
            *
            * Therefore pitch, hump detection and ramp-height estimation
            * use Y.
            */
            pitch_deg = estimate_pitch_deg(dy);

            distance_mm = subsystem4_take_distance_mm();
            if (distance_mm > 0U) {
                movement_hold_samples = S4_MOVEMENT_HOLD_SAMPLES;
            } else if (movement_hold_samples > 0U) {
                movement_hold_samples--;
            }

            completed_hump_peak = 0;

            /* Lift is the safety event (#53): highest classification
               priority. A hump in progress is abandoned. */
            if (update_lift(sample.az_mg, baseline_z, &lift_delta_mg)) {
                motion = IMU_MOTION_LIFTED;
                hump_sum = 0;
                hump_next = 0U;
                hump_count = 0U;
                reset_hump_detector();
                reset_height_tracker();
                tilt_samples = 0U;

                if (!lift_reported) {
                    event = IMU_EVENT_LIFT;
                    lift_reported = TRUE;
                    LOG_INFO(TAG_S4_IMU, "EVENT LIFT z_avg-base=%dmg threshold=%dmg acc=(%d,%d,%d) base_z=%d",
                              lift_delta_mg, S4_LIFT_DELTA_MG,
                              sample.ax_mg, sample.ay_mg, sample.az_mg, baseline_z);
                }
            } else {
                if (hump_count == S4_HUMP_FILTER_SAMPLES) {
                    hump_sum -= hump_samples[hump_next];
                } else {
                    hump_count++;
                }
                hump_samples[hump_next] = dy;
                hump_sum += dy;
                hump_next = (hump_next + 1U) % S4_HUMP_FILTER_SAMPLES;
                W hump_dy = hump_sum / (W)hump_count;

                hump_was_active = (hump_state.phase != HUMP_IDLE);
                if (!hump_was_active && hump_armed &&
                    abs_w(hump_dy) >= S4_HUMP_OPPOSITE_MG) {
                    LOG_INFO(TAG_S4_IMU,
                              "HUMP DEBUG: raw=%d filtered=%d moving=%d dist=%u entry=%u",
                              dy, hump_dy, movement_hold_samples > 0U,
                              distance_mm, hump_state.entry_samples);
                }
                hump_completed = update_hump(
                    hump_dy,
                    movement_hold_samples > 0U,
                    distance_mm,
                    &completed_hump_peak);
                hump_is_active = (hump_state.phase != HUMP_IDLE);

                if (!hump_was_active && hump_is_active) {
                    /*
                     * The entry-confirmation samples are already part of
                     * the uphill climb, so preserve their accumulated rise.
                     */
                    reset_height_tracker();

                    current_height_milli_mm =
                        hump_entry_height_milli_mm;

                    current_height_mm =
                        calibrated_height_mm(current_height_milli_mm);

                    if (current_height_mm > 0) {
                        peak_height_mm = (UW)current_height_mm;
                    }

                    if (hump_entry_height_milli_mm > 0) {
                        height_distance_seen = TRUE;
                    }
                }

                if (hump_is_active) {
                    /*
                     * When the hump starts on this exact cycle, the current
                     * entry sample has already been included above.
                     */
                    if (hump_was_active) {
                        update_height_estimate(distance_mm, hump_dy);
                    }

                    motion = IMU_MOTION_HUMP;
                }

                if (hump_completed) {
                    motion = IMU_MOTION_HUMP;
                    event = IMU_EVENT_HUMP;
                    last_hump_peak = completed_hump_peak;
                    if (height_distance_seen) {
                        if (peak_height_mm > last_ramp_height) {
                            last_ramp_height = peak_height_mm;
                            last_ramp_height_valid = TRUE;
                        }
                        LOG_INFO(TAG_S4_IMU,
                                  "EVENT HUMP height=%u mm highest=%u mm peak=%d mg",
                                  peak_height_mm, last_ramp_height, last_hump_peak);
                    } else {
                        LOG_INFO(TAG_S4_IMU, "EVENT HUMP height=unavailable (no wheel distance) peak=%d mg",
                                  last_hump_peak);
                    }
                } else {
                    /* A normal ramp should not be misclassified as severe tilt. */
                    tilt_now = update_tilt(dx, dy, hump_is_active);

                    if (tilt_now) {
                        W roll_deg = estimate_pitch_deg(dx);
                        motion = IMU_MOTION_TILT;
                        if (!tilt_reported) {
                            event = IMU_EVENT_TILT;
                            tilt_reported = TRUE;
                            LOG_INFO(TAG_S4_IMU,
                                      "EVENT TILT roll=%d deg lateral=%d mg", roll_deg,dx);
                        }
                    }
                }
            }

            tk_get_tim(&cycle_end);
            cycle_ms = cycle_end.lo - cycle_start.lo;
            timing_overrun = (cycle_ms > S4_SAMPLE_PERIOD_MS) ? TRUE : FALSE;

            publish_snapshot(&sample,
                             baseline_x, baseline_y, baseline_z,
                             last_hump_peak,
                             pitch_deg,
                             last_ramp_height,
                             last_ramp_height_valid,
                             motion,
                             event,
                             TRUE,
                             TRUE,
                             cycle_ms,
                             timing_overrun);

            print_periodic_status(&sample,
                                baseline_x,
                                baseline_y,
                                motion,
                                pitch_deg,
                                last_ramp_height,
                                last_ramp_height_valid,
                                local_sample_count);
        }

        /* Maintain the configured interval between task cycles. */
        tk_get_tim(&cycle_end);
        cycle_ms = cycle_end.lo - cycle_start.lo;

        if (cycle_ms < S4_SAMPLE_PERIOD_MS) {
            s4_delay((TMO)(S4_SAMPLE_PERIOD_MS - cycle_ms));
        } else {
            /* Do not print every overrun: USB logging would worsen timing. */
            s4_delay(1);
        }
    }
}
