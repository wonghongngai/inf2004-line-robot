/*
 * task_imu.c
 * Buddy 4 -- 20 ms IMU sampling, calibration and terrain/event detection.
 *
 * Pre-change baseline: no minimum-bit packing is implemented here.
 */

#include "subsystem4.h"
#include "drv_lsm303dlhc.h"

/* Hump detector phases. */
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

LOCAL W abs_w(W value)
{
    return (value < 0) ? -value : value;
}

LOCAL W max3(W a, W b, W c)
{
    W max = a;

    if (b > max) max = b;
    if (c > max) max = c;
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
    hump_state.age_samples = 0;
    hump_state.settle_samples = 0;
}

LOCAL void publish_snapshot(const LSM303_SAMPLE *sample,
                             W baseline_x,
                             W baseline_y,
                             W baseline_z,
                             W hump_peak,
                             IMU_MOTION_STATE motion,
                             IMU_EVENT event,
                             BOOL calibrated,
                             BOOL sensor_ok,
                             UW cycle_ms)
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
    s4_imu_data.motion_state = motion;
    s4_imu_data.event = event;
    s4_imu_data.calibrated = calibrated;
    s4_imu_data.sensor_ok = sensor_ok;
    s4_imu_data.last_cycle_ms = cycle_ms;

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

LOCAL BOOL detect_collision(W dx, W dy, W dz)
{
    W peak_delta;

    peak_delta = max3(abs_w(dx), abs_w(dy), abs_w(dz));
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
        tilt_samples = 0;
        tilt_reported = FALSE;
    }

    if (tilt_samples >= S4_TILT_CONFIRM_SAMPLES) {
        return TRUE;
    }

    return FALSE;
}

/*
 * Simple pre-change hump detector.
 * Assumption: sensor X-axis is approximately aligned with forward travel.
 * The sign depends on physical mounting, so the detector accepts either sign.
 * A hump is treated as a multi-sample pattern: first slope -> opposite slope
 * -> return near the calibrated baseline.
 */
LOCAL BOOL update_hump(W dx, W *completed_peak)
{
    W abs_dx;

    abs_dx = abs_w(dx);

    if (completed_peak != NULL) {
        *completed_peak = 0;
    }

    if (hump_state.phase != HUMP_IDLE) {
        hump_state.age_samples++;
        if (hump_state.age_samples > S4_HUMP_TIMEOUT_SAMPLES) {
            reset_hump_detector();
            return FALSE;
        }
    }

    switch (hump_state.phase) {
    case HUMP_IDLE:
        if (abs_dx >= S4_HUMP_ENTRY_MG) {
            hump_state.phase = HUMP_FIRST_SLOPE;
            hump_state.first_sign = sign_w(dx);
            hump_state.peak_mg = abs_dx;
            hump_state.age_samples = 0;
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
            hump_state.settle_samples = 1;
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
            hump_state.settle_samples = 0;
            hump_state.phase = HUMP_OPPOSITE_SLOPE;
        }
        break;

    default:
        reset_hump_detector();
        break;
    }

    return FALSE;
}

LOCAL void print_periodic_status(const LSM303_SAMPLE *sample,
                                  IMU_MOTION_STATE motion,
                                  UW sample_count)
{
    if ((sample_count % S4_PRINT_PERIOD_SAMPLES) != 0U) {
        return;
    }

    tm_printf((UB *)"Buddy4: n=%u acc=(%d,%d,%d)mg mag=(%d,%d,%d) state=%d\n",
              sample_count,
              sample->ax_mg, sample->ay_mg, sample->az_mg,
              sample->mx_raw, sample->my_raw, sample->mz_raw,
              motion);
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
    W dz;
    W completed_hump_peak;
    W last_hump_peak = 0;
    UW cycle_ms;
    UW local_sample_count = 0;
    BOOL calibrated = FALSE;
    BOOL sensor_ok = FALSE;
    BOOL tilt_now;
    IMU_MOTION_STATE motion;
    IMU_EVENT event;
    ER er;

    (void)stacd;
    (void)exinf;

    cal_state.sum_x = 0;
    cal_state.sum_y = 0;
    cal_state.sum_z = 0;
    cal_state.count = 0;
    tilt_samples = 0;
    tilt_reported = FALSE;
    reset_hump_detector();

    tm_printf((UB *)"Buddy4: IMU task started, period=%d ms\n",
              S4_SAMPLE_PERIOD_MS);

    while (1) {
        tk_get_tim(&cycle_start);

        if (!sensor_ok) {
            er = lsm303dlhc_init();
            if (er < E_OK) {
                record_read_error();
                tk_dly_tsk(200);
                continue;
            }

            sensor_ok = TRUE;
            calibrated = FALSE;
            cal_state.sum_x = 0;
            cal_state.sum_y = 0;
            cal_state.sum_z = 0;
            cal_state.count = 0;
            reset_hump_detector();
            tilt_samples = 0;
            tilt_reported = FALSE;
        }

        er = lsm303dlhc_read(&sample);
        if (er < E_OK) {
            tm_printf((UB *)"Buddy4: IMU read failed, er=%d\n", er);
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

            publish_snapshot(&sample,
                             baseline_x, baseline_y, baseline_z,
                             last_hump_peak,
                             IMU_MOTION_NORMAL,
                             IMU_EVENT_NONE,
                             calibrated,
                             TRUE,
                             0);
        } else {
            dx = sample.ax_mg - baseline_x;
            dy = sample.ay_mg - baseline_y;
            dz = sample.az_mg - baseline_z;

            completed_hump_peak = 0;

            /* Safety event takes precedence over terrain classification. */
            if (detect_collision(dx, dy, dz)) {
                motion = IMU_MOTION_COLLISION;
                event = IMU_EVENT_COLLISION;
                reset_hump_detector();
                tilt_samples = 0;

                tm_printf((UB *)"Buddy4 EVENT: COLLISION delta=(%d,%d,%d)mg\n",
                          dx, dy, dz);
            } else {
                tilt_now = update_tilt(dx, dy);

                if (tilt_now) {
                    motion = IMU_MOTION_TILT;
                    if (!tilt_reported) {
                        event = IMU_EVENT_TILT;
                        tilt_reported = TRUE;
                        tm_printf((UB *)"Buddy4 EVENT: TILT delta=(%d,%d)mg\n",
                                  dx, dy);
                    }
                } else if (update_hump(dx, &completed_hump_peak)) {
                    motion = IMU_MOTION_HUMP;
                    event = IMU_EVENT_HUMP;
                    last_hump_peak = completed_hump_peak;

                    tm_printf((UB *)"Buddy4 EVENT: HUMP peak=%d mg\n",
                              last_hump_peak);
                } else if (hump_state.phase != HUMP_IDLE) {
                    motion = IMU_MOTION_HUMP;
                }
            }

            tk_get_tim(&cycle_end);
            cycle_ms = cycle_end.lo - cycle_start.lo;

            publish_snapshot(&sample,
                             baseline_x, baseline_y, baseline_z,
                             last_hump_peak,
                             motion,
                             event,
                             TRUE,
                             TRUE,
                             cycle_ms);

            print_periodic_status(&sample, motion, local_sample_count);
        }

        /* Maintain approximately 20 ms between loop starts. */
        tk_get_tim(&cycle_end);
        cycle_ms = cycle_end.lo - cycle_start.lo;

        if (cycle_ms < S4_SAMPLE_PERIOD_MS) {
            tk_dly_tsk(S4_SAMPLE_PERIOD_MS - cycle_ms);
        } else {
            tm_printf((UB *)"Buddy4 WARN: cycle=%u ms exceeds %d ms period\n",
                      cycle_ms, S4_SAMPLE_PERIOD_MS);
            tk_dly_tsk(1);
        }
    }
}
