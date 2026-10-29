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
    W peak_mg;
    UW age_samples;
    UW settle_samples;
} HUMP_STATE;

LOCAL CAL_STATE cal_state;
LOCAL HUMP_STATE hump_state;
LOCAL UW tilt_samples;
LOCAL BOOL tilt_reported;

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

LOCAL W max3(W a, W b, W c)
{
    W max = a;

    if (b > max) {
        max = b;
    }
    if (c > max) {
        max = c;
    }
    return max;
}

LOCAL INT sign_w(W value)
{
    return (value >= 0) ? 1 : -1;
}

LOCAL void reset_hump_detector(void)
{
    hump_state.phase = HUMP_IDLE;
    hump_state.first_sign = 0;
    hump_state.peak_mg = 0;
    hump_state.age_samples = 0U;
    hump_state.settle_samples = 0U;
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
 * Assumes the IMU X axis is aligned approximately with forward travel.
 * The table maps |Ax-baseline| in mg to the nearest 5-degree pitch angle.
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

LOCAL void update_height_estimate(UW distance_mm, W dx)
{
    W aligned_dx;

    if (distance_mm == 0U || hump_state.first_sign == 0) {
        return;
    }

    /* Make the first/up slope positive regardless of IMU mounting sign. */
    aligned_dx = (W)(dx * hump_state.first_sign);
    if (aligned_dx > 1000) aligned_dx = 1000;
    if (aligned_dx < -1000) aligned_dx = -1000;
    height_distance_seen = TRUE;

    /* Accumulate before rounding, on ascent AND descent. */
    current_height_milli_mm += (int64_t)distance_mm * aligned_dx;
    if (current_height_milli_mm < 0) current_height_milli_mm = 0;
    if (current_height_milli_mm > (int64_t)S4_MAX_RAMP_HEIGHT_MM * 1000)
        current_height_milli_mm = (int64_t)S4_MAX_RAMP_HEIGHT_MM * 1000;
    current_height_mm = (W)(current_height_milli_mm / 1000);

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

    if (timing_overrun) {
        s4_imu_data.timing_overruns++;
    }

    if (sample != NULL && sensor_ok) {
        s4_imu_data.sample_count++;
    }

    tk_unl_mtx(mtx_imu_data);
}

LOCAL void record_read_error(void)
{
    if (tk_loc_mtx(mtx_imu_data, TMO_FEVR) >= E_OK) {
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

    tm_printf((UB *)"Buddy4: calibration complete base=(%d,%d,%d) mg\n",
              *baseline_x, *baseline_y, *baseline_z);

    return TRUE;
}

/* Collision = sudden sample-to-sample acceleration change. */
LOCAL BOOL detect_collision(const LSM303_SAMPLE *current,
                            const LSM303_SAMPLE *previous,
                            W *measured_delta_mg)
{
    W dax;
    W day;
    W daz;
    W peak_delta;

    dax = current->ax_mg - previous->ax_mg;
    day = current->ay_mg - previous->ay_mg;
    daz = current->az_mg - previous->az_mg;

    peak_delta = max3(abs_w(dax), abs_w(day), abs_w(daz));
    if (measured_delta_mg != NULL) *measured_delta_mg = peak_delta;
    return (peak_delta >= S4_COLLISION_DELTA_MG) ? TRUE : FALSE;
}

LOCAL BOOL update_tilt(W dx, W dy)
{
    BOOL beyond_threshold;

    beyond_threshold = (abs_w(dx) >= S4_TILT_DELTA_MG ||
                        abs_w(dy) >= S4_TILT_DELTA_MG);

    if (beyond_threshold) {
        if (tilt_samples < S4_TILT_CONFIRM_SAMPLES) {
            tilt_samples++;
        }
    } else {
        tilt_samples = 0U;
        tilt_reported = FALSE;
    }

    return (tilt_samples >= S4_TILT_CONFIRM_SAMPLES) ? TRUE : FALSE;
}

/* Multi-sample ramp pattern: up slope -> opposite slope -> flat again. */
LOCAL BOOL update_hump(W dx, W *completed_peak)
{
    W abs_dx = abs_w(dx);

    if (completed_peak != NULL) {
        *completed_peak = 0;
    }

    if (hump_state.phase != HUMP_IDLE) {
        hump_state.age_samples++;
        if (hump_state.age_samples > S4_HUMP_TIMEOUT_SAMPLES) {
            reset_hump_detector();
            reset_height_tracker();
            return FALSE;
        }
    }

    switch (hump_state.phase) {
    case HUMP_IDLE:
        if (abs_dx >= S4_HUMP_ENTRY_MG) {
            hump_state.phase = HUMP_FIRST_SLOPE;
            hump_state.first_sign = sign_w(dx);
            hump_state.peak_mg = abs_dx;
            hump_state.age_samples = 0U;
        }
        break;

    case HUMP_FIRST_SLOPE:
        if (abs_dx > hump_state.peak_mg) {
            hump_state.peak_mg = abs_dx;
        }

        if (sign_w(dx) != hump_state.first_sign &&
            abs_dx >= S4_HUMP_OPPOSITE_MG) {
            hump_state.phase = HUMP_OPPOSITE_SLOPE;
        }
        break;

    case HUMP_OPPOSITE_SLOPE:
        if (abs_dx > hump_state.peak_mg) {
            hump_state.peak_mg = abs_dx;
        }

        if (abs_dx <= S4_HUMP_SETTLE_MG) {
            hump_state.phase = HUMP_SETTLING;
            hump_state.settle_samples = 1U;
        }
        break;

    case HUMP_SETTLING:
        if (abs_dx <= S4_HUMP_SETTLE_MG) {
            hump_state.settle_samples++;
            if (hump_state.settle_samples >= S4_HUMP_SETTLE_SAMPLES) {
                if (completed_peak != NULL) {
                    *completed_peak = hump_state.peak_mg;
                }
                reset_hump_detector();
                return TRUE;
            }
        } else {
            hump_state.settle_samples = 0U;
            hump_state.phase = HUMP_OPPOSITE_SLOPE;
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
                                 IMU_MOTION_STATE motion,
                                 W pitch_deg,
                                 UW ramp_height_mm,
                                 BOOL ramp_height_valid,
                                 UW sample_count)
{
    if ((sample_count % S4_PRINT_PERIOD_SAMPLES) != 0U) {
        return;
    }

    tm_printf((UB *)
              "Buddy4: n=%u acc=(%d,%d,%d)mg mag=(%d,%d,%d) pitch=%d height=%umm height_valid=%d state=%d\n",
              sample_count,
              sample->ax_mg, sample->ay_mg, sample->az_mg,
              sample->mx_raw, sample->my_raw, sample->mz_raw,
              pitch_deg, ramp_height_mm, ramp_height_valid, motion);
}

EXPORT void imu_task(INT stacd, void *exinf)
{
    LSM303_SAMPLE sample;
    LSM303_SAMPLE previous_sample;
    SYSTIM cycle_start;
    SYSTIM cycle_end;
    W baseline_x = 0;
    W baseline_y = 0;
    W baseline_z = 0;
    W dx;
    W dy;
    W completed_hump_peak;
    W last_hump_peak = 0;
    W pitch_deg = 0;
    W collision_delta_mg = 0;
    UW last_ramp_height = 0U;
    BOOL last_ramp_height_valid = FALSE;
    UW distance_mm;
    UW cycle_ms;
    UW local_sample_count = 0U;
    BOOL calibrated = FALSE;
    BOOL sensor_ok = FALSE;
    BOOL have_previous = FALSE;
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
    reset_hump_detector();
    reset_height_tracker();

    tm_printf((UB *)"Buddy4: IMU task started, period=%d ms\n",
              S4_SAMPLE_PERIOD_MS);

    while (1) {
        tk_get_tim(&cycle_start);
        timing_overrun = FALSE;

        if (!sensor_ok) {
            er = lsm303dlhc_init();
            if (er < E_OK) {
                record_read_error();
                tk_dly_tsk(200);
                continue;
            }

            sensor_ok = TRUE;
            calibrated = FALSE;
            have_previous = FALSE;
            cal_state.sum_x = 0;
            cal_state.sum_y = 0;
            cal_state.sum_z = 0;
            cal_state.count = 0U;
            tilt_samples = 0U;
            tilt_reported = FALSE;
            reset_hump_detector();
            reset_height_tracker();
        }

        er = lsm303dlhc_read(&sample);
        if (er < E_OK) {
            sensor_ok = FALSE;
            record_read_error();
            tk_dly_tsk(200);
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

            previous_sample = sample;
            have_previous = TRUE;

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
            pitch_deg = estimate_pitch_deg(dx);
            distance_mm = subsystem4_take_distance_mm();
            completed_hump_peak = 0;

            /* Safety event has highest classification priority. */
            if (have_previous && detect_collision(&sample, &previous_sample, &collision_delta_mg)) {
                motion = IMU_MOTION_COLLISION;
                event = IMU_EVENT_COLLISION;
                reset_hump_detector();
                reset_height_tracker();
                tilt_samples = 0U;

                tm_printf((UB *)"Buddy4 EVENT: COLLISION delta=%dmg threshold=%dmg previous=(%d,%d,%d) current=(%d,%d,%d)\n",
                          collision_delta_mg, S4_COLLISION_DELTA_MG,
                          previous_sample.ax_mg, previous_sample.ay_mg, previous_sample.az_mg,
                          sample.ax_mg, sample.ay_mg, sample.az_mg);
            } else {
                hump_was_active = (hump_state.phase != HUMP_IDLE);
                hump_completed = update_hump(dx, &completed_hump_peak);
                hump_is_active = (hump_state.phase != HUMP_IDLE);

                if (!hump_was_active && hump_is_active) {
                    reset_height_tracker();
                }

                if (hump_is_active) {
                    update_height_estimate(distance_mm, dx);
                    motion = IMU_MOTION_HUMP;
                }

                if (hump_completed) {
                    motion = IMU_MOTION_HUMP;
                    event = IMU_EVENT_HUMP;
                    last_hump_peak = completed_hump_peak;
                    if (height_distance_seen) {
                        if (peak_height_mm > last_ramp_height)
                            last_ramp_height = peak_height_mm;
                        last_ramp_height_valid = TRUE;
                        tm_printf((UB *)
                                  "Buddy4 EVENT: HUMP height=%u mm highest=%u mm peak=%d mg\n",
                                  peak_height_mm, last_ramp_height, last_hump_peak);
                    } else {
                        tm_printf((UB *)"Buddy4 EVENT: HUMP height=unavailable (no wheel distance) peak=%d mg\n",
                                  last_hump_peak);
                    }
                } else {
                    /* A normal ramp should not be misclassified as severe tilt. */
                    tilt_now = update_tilt(dx, dy);

                    if (tilt_now && !hump_is_active) {
                        motion = IMU_MOTION_TILT;
                        if (!tilt_reported) {
                            event = IMU_EVENT_TILT;
                            tilt_reported = TRUE;
                            tm_printf((UB *)
                                      "Buddy4 EVENT: TILT pitch=%d deg\n",
                                      pitch_deg);
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
                                  motion,
                                  pitch_deg,
                                  last_ramp_height,
                                  last_ramp_height_valid,
                                  local_sample_count);
        }

        previous_sample = sample;
        have_previous = TRUE;

        /* Maintain approximately 20 ms between task cycles. */
        tk_get_tim(&cycle_end);
        cycle_ms = cycle_end.lo - cycle_start.lo;

        if (cycle_ms < S4_SAMPLE_PERIOD_MS) {
            tk_dly_tsk(S4_SAMPLE_PERIOD_MS - cycle_ms);
        } else {
            /* Do not print every overrun: USB logging would worsen timing. */
            tk_dly_tsk(1);
        }
    }
}
