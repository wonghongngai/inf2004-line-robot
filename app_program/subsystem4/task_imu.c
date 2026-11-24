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
#if defined(TK_SUPPORT_SMP) && TK_SUPPORT_SMP
#include <smp_lock.h> /* kernel/knlinc: big-kernel-lock audit for the FATAL report */
#endif
/* Sample counts at S4_SAMPLE_PERIOD_MS = 20 ms (halved 20 Nov from the 10 ms values). */
#define S4_HUMP_REARM_FLAT_SAMPLES    25U /* 500 ms */
#define S4_HUMP_ENTRY_CONFIRM_SAMPLES 10U /* 200 ms (was 3: noise passed it) */
#define S4_MOVEMENT_HOLD_SAMPLES      5U  /* 100 ms */
/* A hump may only start while the wheels turned this sample or the one
   before: braking at a stop jolts the forward axis while the 100 ms
   movement hold still says "moving" (4 false humps at PAUSE, 23 Nov). */
#define S4_HUMP_ENTRY_MOVING_SAMPLES  2U
/* Pulling away from rest tips the robot back: the 200 ms average went to
   -40..-59 mg for ~14 samples after START and gave false humps on 2 of 3
   flat runs (23 Nov, 17:57 and 17:58). No hump may start for 500 ms after
   the wheels start turning from a standstill (no distance for the
   100 ms movement hold). */
#define S4_HUMP_START_BLANK_SAMPLES   25U /* 500 ms */
#define S4_HUMP_FILTER_SAMPLES        10U /* 200 ms moving average (was 3) */
#define S4_HUMP_EXIT_FLAT_MG          S4_HUMP_SETTLE_MG
#define S4_HUMP_EXIT_FLAT_SAMPLES \
    10U /* 200 ms (was 4: noise dips ended humps mid-climb) */
/*
 * Height factor. 25/11 was fitted when the detector measured only the middle
 * of the climb (it confirmed late and ended early); with the climb counted
 * from level ground to level ground the raw rise of the 10 mm ramp already
 * came to 7.6-9.4 mm before the old events fired. Reset to 1/1, then refitted
 * on the robot (#27 step 5, 23 Nov): three whole climbs of the 10 mm ramp
 * read 6, 6, 8 mm at 1/1 (17:54-17:57) and five earlier climbs, cut short
 * by the recording, at least 7, 7, 8, 9, 7 mm. Mean 7.25 mm -> 1.38; 4/3
 * puts all eight within +-2 mm in the PC replay (3/2, fitted on the last
 * three alone, is +4 on one earlier climb). S4-U10 is scored on fresh
 * passes with this factor.
 */
#define S4_HEIGHT_CAL_NUM 4
#define S4_HEIGHT_CAL_DEN 3
typedef enum
{
    HUMP_IDLE = 0,
    HUMP_FIRST_SLOPE,
    HUMP_OPPOSITE_SLOPE,
    HUMP_SETTLING
} HUMP_PHASE;

typedef struct
{
    W  sum_x;
    W  sum_y;
    W  sum_z;
    UW count;
} CAL_STATE;

typedef struct
{
    HUMP_PHASE phase;
    INT        first_sign;
    BOOL       descent_started;
    W          peak_mg;
    UW         age_samples;
    UW         settle_samples;

    /* Entry confirmation prevents short vibrations from becoming humps. */
    INT entry_sign;
    UW  entry_samples;
} HUMP_STATE;

LOCAL CAL_STATE  g_cal_state;
LOCAL HUMP_STATE g_hump_state;
LOCAL UW         g_tilt_samples;
LOCAL BOOL       g_b_tilt_reported;
LOCAL BOOL       g_b_hump_armed;
LOCAL UW         g_hump_rearm_count;
LOCAL UW         g_movement_hold_samples;
LOCAL UW         g_entry_moving_samples; /* S4_HUMP_ENTRY_MOVING_SAMPLES hold */
LOCAL UW g_start_blank_samples; /* S4_HUMP_START_BLANK_SAMPLES after a start */
LOCAL int64_t
    g_hump_entry_height_milli_mm; /* climb since level ground, before entry */
LOCAL INT g_hump_climb_sign;      /* its direction, 0 on level ground */

/* Lift check (#53): moving average of the clipped Z deviation and the
 * consecutive-sample count. */
LOCAL W    g_lift_samples[S4_LIFT_FILTER_SAMPLES];
LOCAL W    g_lift_sum;
LOCAL UW   g_lift_next;
LOCAL UW   g_lift_count;
LOCAL UW   g_lift_confirm;
LOCAL BOOL g_b_lift_reported;

/* Height estimator state for one ramp crossing. */
LOCAL W  g_current_height_mm;
LOCAL UW g_peak_height_mm;
/* Retain fractional mm: mm * mg / 1000 loses small rises if each sample
 * is rounded separately. 1000 units here represent one millimetre. */
LOCAL int64_t g_current_height_milli_mm;
LOCAL BOOL    g_b_height_distance_seen;

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
    g_hump_state.phase           = HUMP_IDLE;
    g_hump_state.first_sign      = 0;
    g_hump_state.descent_started = FALSE;
    g_hump_state.peak_mg         = 0;
    g_hump_state.age_samples     = 0U;
    g_hump_state.settle_samples  = 0U;

    g_hump_state.entry_sign      = 0;
    g_hump_state.entry_samples   = 0U;
    g_hump_entry_height_milli_mm = 0;
    g_hump_climb_sign            = 0;
}

LOCAL void reset_height_tracker(void)
{
    g_current_height_mm       = 0;
    g_peak_height_mm          = 0U;
    g_current_height_milli_mm = 0;
    g_b_height_distance_seen  = FALSE;
}

/*
 * Hump capture build (#27 step 2, `sh build.sh integrated S4_TRACE=1`).
 *
 * Every calibrated sample goes into a RAM ring. When a hump measurement ends
 * (event, timeout, or a lift) the last S4_TRACE_LEN samples are copied out
 * and printed S4_TRACE_PER_LINE per cycle, so nothing is printed while the
 * hump is being measured. While moving on level ground a noise line is
 * printed every S4_PRINT_PERIOD_SAMPLES samples (S4-U08 driving noise).
 * Off (0) in every normal build; the per-sample HUMP ACTIVE / HUMP DEBUG
 * lines are left out of trace builds because the trace already holds them.
 *
 *   HT#k <why> len=N cols=n,ph,mv,raw,f,d,h,s,cyc
 *   HT#k n,ph,mv,raw,f,d,h,s,cyc;...    (oldest first)
 *   HT noise n=50 f=min..max |f|=mean raw=min..max
 *
 * n = sample number (low 16 bits), ph = hump phase after the sample
 * (0 idle, 1 first slope, 2 opposite, 3 settling, 8 not armed, 9 lifted),
 * mv = moving, raw = forward dy (mg), f = filtered dy (or the lift deviation
 * when ph = 9), d = wheel distance this sample (mm), h = peak height (mm),
 * s = settle count, cyc = previous cycle time (ms).
 */
#ifndef S4_HUMP_TRACE
#define S4_HUMP_TRACE 0
#endif
#if S4_HUMP_TRACE
#define S4_TRACE_LEN        128U /* 2.56 s: covers the 1.5 s timeout + entry */
#define S4_TRACE_PER_LINE   4U
#define S4_TRACE_PH_UNARMED 8U
#define S4_TRACE_PH_LIFTED  9U

typedef struct
{
    uint16_t n;
    uint8_t  phase;
    uint8_t  moving;
    int16_t  raw;
    int16_t  filt;
    uint16_t dist;
    uint8_t  height;
    uint8_t  settle;
    uint8_t  cyc;
} S4_TRACE_REC;

LOCAL S4_TRACE_REC g_trace_ring[S4_TRACE_LEN];
LOCAL UW           g_trace_next;
LOCAL UW           g_trace_count;
LOCAL S4_TRACE_REC g_trace_out[S4_TRACE_LEN];
LOCAL UW           g_trace_out_len;
LOCAL UW           g_trace_out_pos;
LOCAL UW           g_trace_id;
LOCAL UW           g_trace_last_cycle_ms;

LOCAL W  g_noise_fmin, g_noise_fmax, g_noise_rmin, g_noise_rmax;
LOCAL W  g_noise_abs_sum;
LOCAL UW g_noise_n;

LOCAL int16_t trace_clamp16(W value)
{
    return (int16_t)((value > 32767) ? 32767
                                     : ((value < -32768) ? -32768 : value));
}

LOCAL uint8_t trace_clamp8(UW value)
{
    return (uint8_t)((value > 255U) ? 255U : value);
}

LOCAL void trace_record(UW sample, UW phase, BOOL b_moving, W raw, W filt,
                        UW dist)
{
    S4_TRACE_REC * p_rec = &g_trace_ring[g_trace_next];

    p_rec->n      = (uint16_t)sample;
    p_rec->phase  = (uint8_t)phase;
    p_rec->moving = b_moving ? 1U : 0U;
    p_rec->raw    = trace_clamp16(raw);
    p_rec->filt   = trace_clamp16(filt);
    p_rec->dist   = (uint16_t)((dist > 65535U) ? 65535U : dist);
    p_rec->height = trace_clamp8(g_peak_height_mm);
    p_rec->settle = trace_clamp8(g_hump_state.settle_samples);
    p_rec->cyc    = trace_clamp8(g_trace_last_cycle_ms);

    g_trace_next = (g_trace_next + 1U) % S4_TRACE_LEN;
    if (g_trace_count < S4_TRACE_LEN)
    {
        g_trace_count++;
    }
}

/* Copy the ring out for printing; a dump still printing is replaced. */
LOCAL void trace_freeze(const char * p_why)
{
    UW   idx;
    UW   start = (g_trace_next + S4_TRACE_LEN - g_trace_count) % S4_TRACE_LEN;
    BOOL b_cut = (g_trace_out_pos < g_trace_out_len) ? TRUE : FALSE;

    for (idx = 0U; idx < g_trace_count; idx++)
    {
        g_trace_out[idx] = g_trace_ring[(start + idx) % S4_TRACE_LEN];
    }
    g_trace_out_len = g_trace_count;
    g_trace_out_pos = 0U;
    g_trace_id++;

    LOG_INFO(TAG_S4_IMU, "HT#%u %s len=%u cols=n,ph,mv,raw,f,d,h,s,cyc%s",
             g_trace_id, p_why, g_trace_out_len,
             b_cut ? " (previous dump cut short)" : "");
}

/* Print at most one line of the frozen trace per cycle. */
LOCAL void trace_dump_step(void)
{
    /* One record is at most 9 fields x 6 chars + 9 separators = 63. */
    static UB            line[32 + S4_TRACE_PER_LINE * 64U];
    const S4_TRACE_REC * p_rec;
    INT                  len;
    UW                   rec_idx;

    if (g_trace_out_pos >= g_trace_out_len)
    {
        return;
    }

    /* Built in one buffer and printed once, so other tasks cannot split it. */
    len = tm_sprintf(line, (UB *)"[" TAG_S4_IMU "] HT#%u ", g_trace_id);
    for (rec_idx = 0U;
         rec_idx < S4_TRACE_PER_LINE && g_trace_out_pos < g_trace_out_len;
         rec_idx++)
    {
        p_rec = &g_trace_out[g_trace_out_pos++];
        len += tm_sprintf(&line[len], (UB *)"%s%u,%u,%u,%d,%d,%u,%u,%u,%u",
                          (rec_idx == 0U) ? "" : ";", (UW)p_rec->n,
                          (UW)p_rec->phase, (UW)p_rec->moving, (W)p_rec->raw,
                          (W)p_rec->filt, (UW)p_rec->dist, (UW)p_rec->height,
                          (UW)p_rec->settle, (UW)p_rec->cyc);
    }
    tm_printf((UB *)"%s\n", line);
}

/* Level-ground noise while moving (S4-U08); partial windows are dropped. */
LOCAL void trace_noise(BOOL b_moving, BOOL b_hump_idle, W raw, W filt)
{
    W abs_filt = abs_w(filt);

    if (!b_moving || !b_hump_idle)
    {
        g_noise_n = 0U;
        return;
    }
    if (g_noise_n == 0U)
    {
        g_noise_fmin = g_noise_fmax = filt;
        g_noise_rmin = g_noise_rmax = raw;
        g_noise_abs_sum             = 0;
    }
    if (filt < g_noise_fmin)
    {
        g_noise_fmin = filt;
    }
    if (filt > g_noise_fmax)
    {
        g_noise_fmax = filt;
    }
    if (raw < g_noise_rmin)
    {
        g_noise_rmin = raw;
    }
    if (raw > g_noise_rmax)
    {
        g_noise_rmax = raw;
    }
    g_noise_abs_sum += abs_filt;
    g_noise_n++;

    if (g_noise_n >= S4_PRINT_PERIOD_SAMPLES)
    {
        LOG_INFO(TAG_S4_IMU, "HT noise n=%u f=%d..%d |f|=%d raw=%d..%d",
                 g_noise_n, g_noise_fmin, g_noise_fmax,
                 (W)(g_noise_abs_sum / (W)g_noise_n), g_noise_rmin,
                 g_noise_rmax);
        g_noise_n = 0U;
    }
}
#endif /* S4_HUMP_TRACE */

/*
 * Estimate pitch for logging/telemetry without floating-point trigonometry.
 * Uses the IMU Y axis as the forward/ramp axis for the current mounting.
 * The table maps |forward-baseline| in mg to the nearest 5-degree pitch angle.
 */
LOCAL W estimate_pitch_deg(W forward_mg)
{
    static const W sin_table_mg[] = {0,   87,  174, 259, 342, 423, 500,
                                     574, 643, 707, 766, 819, 866, 906,
                                     940, 966, 985, 996, 1000};
    W              value;
    INT            sign;
    UW             idx;

    sign  = sign_w(forward_mg);
    value = abs_w(forward_mg);

    if (value >= 1000)
    {
        return (W)(90 * sign);
    }

    for (idx = 1U; idx < 19U; idx++)
    {
        if (value < sin_table_mg[idx])
        {
            W lower = sin_table_mg[idx - 1U];
            W upper = sin_table_mg[idx];

            if ((value - lower) < (upper - value))
            {
                return (W)((INT)(idx - 1U) * 5 * sign);
            }
            return (W)((INT)idx * 5 * sign);
        }
    }

    return (W)(90 * sign);
}

LOCAL W calibrated_height_mm(int64_t raw_milli_mm)
{
    int64_t calibrated_milli_mm;

    if (raw_milli_mm <= 0)
    {
        return 0;
    }

    calibrated_milli_mm =
        (raw_milli_mm * S4_HEIGHT_CAL_NUM) / S4_HEIGHT_CAL_DEN;

    /* Cap after calibration: telemetry carries 0-255 mm (8 bits). */
    if (calibrated_milli_mm > (int64_t)S4_MAX_RAMP_HEIGHT_MM * 1000)
    {
        calibrated_milli_mm = (int64_t)S4_MAX_RAMP_HEIGHT_MM * 1000;
    }

    return (W)(calibrated_milli_mm / 1000);
}

LOCAL void update_height_estimate(UW distance_mm, W forward_mg)
{
    W aligned_forward;

    if (distance_mm == 0U || g_hump_state.first_sign == 0)
    {
        return;
    }

    /*
     * first_sign is the uphill direction.
     * On the current mounting, nose-up is negative Y, so a negative
     * forward value with first_sign == -1 becomes a positive rise.
     */
    aligned_forward = (W)(forward_mg * g_hump_state.first_sign);

    if (aligned_forward > 1000)
    {
        aligned_forward = 1000;
    }

    /*
     * We report maximum vertical rise. Once the robot is travelling
     * down the opposite side, do not subtract that descent from the
     * height already accumulated on the climb.
     */
    if (aligned_forward <= 0)
    {
        return;
    }

    g_b_height_distance_seen = TRUE;

    /*
     * delta_height ~= distance_mm * sin(theta)
     *              ~= distance_mm * aligned_forward / 1000
     *
     * Keep milli-mm precision. 1000 stored units = 1 mm.
     */
    g_current_height_milli_mm +=
        (int64_t)distance_mm * (int64_t)aligned_forward;

    if (g_current_height_milli_mm > (int64_t)S4_MAX_RAMP_HEIGHT_MM * 1000)
    {
        g_current_height_milli_mm = (int64_t)S4_MAX_RAMP_HEIGHT_MM * 1000;
    }

    g_current_height_mm = calibrated_height_mm(g_current_height_milli_mm);

    if ((UW)g_current_height_mm > g_peak_height_mm)
    {
        g_peak_height_mm = (UW)g_current_height_mm;
    }
}
LOCAL void publish_snapshot(const LSM303_SAMPLE * p_sample, W baseline_x,
                            W baseline_y, W baseline_z, W hump_peak,
                            W pitch_deg, UW ramp_height_mm,
                            BOOL b_ramp_height_valid, IMU_MOTION_STATE motion,
                            IMU_EVENT event, BOOL b_calibrated,
                            BOOL b_sensor_ok, UW cycle_ms,
                            BOOL b_timing_overrun)
{
    ER ercd;

    ercd = tk_loc_mtx(g_mtx_imu_data, TMO_FEVR);
    if (ercd < E_OK)
    {
        return;
    }

    if (p_sample != NULL)
    {
        g_s4_imu_data.ax_mg  = p_sample->ax_mg;
        g_s4_imu_data.ay_mg  = p_sample->ay_mg;
        g_s4_imu_data.az_mg  = p_sample->az_mg;
        g_s4_imu_data.mx_raw = p_sample->mx_raw;
        g_s4_imu_data.my_raw = p_sample->my_raw;
        g_s4_imu_data.mz_raw = p_sample->mz_raw;
    }

    g_s4_imu_data.baseline_ax_mg    = baseline_x;
    g_s4_imu_data.baseline_ay_mg    = baseline_y;
    g_s4_imu_data.baseline_az_mg    = baseline_z;
    g_s4_imu_data.hump_peak_mg      = hump_peak;
    g_s4_imu_data.pitch_deg         = pitch_deg;
    g_s4_imu_data.ramp_height_mm    = ramp_height_mm;
    g_s4_imu_data.ramp_height_valid = b_ramp_height_valid;
    g_s4_imu_data.motion_state      = motion;
    g_s4_imu_data.event             = event;
    g_s4_imu_data.calibrated        = b_calibrated;
    g_s4_imu_data.sensor_ok         = b_sensor_ok;
    g_s4_imu_data.last_cycle_ms     = cycle_ms;
    if (cycle_ms > g_s4_imu_data.max_cycle_ms)
    {
        g_s4_imu_data.max_cycle_ms = cycle_ms;
    }

    if (b_timing_overrun)
    {
        g_s4_imu_data.timing_overruns++;
    }

    if (p_sample != NULL && b_sensor_ok)
    {
        g_s4_imu_data.sample_count++;
    }

    tk_unl_mtx(g_mtx_imu_data);
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
/*
 * Back-off and recovery (S4-U17, robot log 22 Nov 2026 13:50: 10 674 lines of
 * "accelerometer not responding, er=-25" from an image without the guard
 * above). Each consecutive failure doubles the retry delay, from one sample
 * period up to S4_RETRY_MAX_MS; every S4_RECOVER_EVERY_FAILS-th failure also
 * frees the I2C bus (9 clocks + STOP). Not for E_CTX: that is the kernel
 * refusing to block, not the bus. The streak ends at the first good read.
 */
#define S4_RETRY_MIN_MS        S4_SAMPLE_PERIOD_MS
#define S4_RETRY_MAX_MS        1000U
#define S4_RECOVER_EVERY_FAILS 3U
#define S4_ERROR_REPORT_MS     1000U
#define S4_ERROR_MTX_TMO_MS    20

/* Weak: links in modes that build S4 without the S2 motor driver. */
extern void drv_motor_stop(void) __attribute__((weak));

/*
 * Free-running 1 MHz hardware timer: keeps counting when the kernel tick
 * cannot reach this core, so the rate limit and the fallback wait still
 * work on a wedged core. The host test supplies its own.
 */
#ifndef S4_HW_NOW_US
#define S4_HW_NOW_US() in_w(TIMER_TIMERAWL)
#endif

LOCAL UW   g_err_unreported;
LOCAL UW   g_err_last_report_us;
LOCAL BOOL g_b_err_reported;
LOCAL BOOL g_b_wedge_reported;
LOCAL BOOL g_b_wedge_followup_done;
LOCAL UW   g_wedge_report_us;
LOCAL UW   g_fail_streak;
LOCAL UW   g_bus_recoveries;

/* Print the first failure at once, then at most one line per second. */
LOCAL void report_error(const char * p_what, ER ercd)
{
    UW now = S4_HW_NOW_US();

    g_err_unreported++;
    if (g_b_err_reported &&
        (UW)(now - g_err_last_report_us) < S4_ERROR_REPORT_MS * 1000U)
    {
        return;
    }
    LOG_ERR(TAG_S4_IMU,
            "IMU %s failed, er=%d (%u failures since last report, streak %u, "
            "bus recoveries %u)",
            p_what, ercd, g_err_unreported, g_fail_streak, g_bus_recoveries);
    g_err_unreported     = 0U;
    g_err_last_report_us = now;
    g_b_err_reported     = TRUE;
}

/* Retry delay after the n-th consecutive failure (n >= 1). */
LOCAL UW s4_retry_delay_ms(UW failures)
{
    UW delay_ms = S4_RETRY_MIN_MS;

    while (failures > 1U && delay_ms < S4_RETRY_MAX_MS)
    {
        delay_ms *= 2U;
        failures--;
    }
    return (delay_ms > S4_RETRY_MAX_MS) ? S4_RETRY_MAX_MS : delay_ms;
}

/*
 * Big-kernel-lock state (kernel/tkernel/smp_lock.c), printed with the FATAL
 * line and once more S4_WEDGE_FOLLOWUP_MS later. Robot log 22 Nov 14:14
 * (9564a72): FATAL with sysstat=0x3 primask=1 run=5 sched=7, i.e. this task
 * came back from a kernel call with PRIMASK still set while a higher-priority
 * task was waiting. A knl_kernel_lock_leave() without a matching enter()
 * returns early without restoring PRIMASK and is counted in illegal_unlocks,
 * so a non-zero count here points at that path. A large max_hold explains
 * late control steps after the recovery.
 */
#define S4_WEDGE_FOLLOWUP_MS 5000U

LOCAL void report_kernel_lock(const char * p_label)
{
#if defined(TK_SUPPORT_SMP) && TK_SUPPORT_SMP
    KNL_LOCK_DIAG diag;

    knl_kernel_lock_audit(&diag);
    LOG_ERR(TAG_S4_IMU,
            "%s: tid=%d bkl owner=%u depth=%u/%u illegal_unlocks=%u "
            "illegal_suspends=%u dead_dispatch=%u max_hold=%u us max_wait=%u "
            "us max_depth=%u",
            p_label, (INT)tk_get_tid(), diag.owner, diag.depth[0],
            diag.depth[1], diag.illegal_unlocks, diag.illegal_suspends,
            diag.dead_ctxtsk_dispatches, diag.max_hold_us, diag.max_wait_us,
            diag.max_depth);
#else
    (void)p_label;
#endif
}

/* One more look at the kernel state a few seconds after the FATAL line. */
LOCAL void wedge_followup(void)
{
    if (g_b_wedge_reported && !g_b_wedge_followup_done &&
        (UW)(S4_HW_NOW_US() - g_wedge_report_us) >=
            S4_WEDGE_FOLLOWUP_MS * 1000U)
    {
        g_b_wedge_followup_done = TRUE;
        report_kernel_lock("FATAL follow-up");
    }
}

/* The kernel refused to block this task: its core is wedged. */
LOCAL void handle_unblockable_core(ER ercd)
{
    T_RSYS rsys;
    UW     primask = get_primask();
    UINT   sysstat = (tk_ref_sys(&rsys) >= E_OK) ? rsys.sysstat : 0xFFU;

    if (drv_motor_stop != NULL)
    {
        drv_motor_stop(); /* register writes only: works with no kernel */
    }
    if (!g_b_wedge_reported)
    {
        g_b_wedge_reported = TRUE;
        LOG_ERR(TAG_S4_IMU,
                "FATAL core %d cannot block (er=%d sysstat=0x%x primask=%u "
                "run=%d sched=%d); motors stopped",
                tk_get_prc(), ercd, sysstat, primask, (INT)rsys.runtskid,
                (INT)rsys.schedtskid);
        report_kernel_lock("FATAL detail");
        g_wedge_report_us = S4_HW_NOW_US();
    }
    /* This task holds no lock here, so a set PRIMASK was leaked by someone
       else. Unmask so the tick and PendSV run and the VC can latch its stop. */
    if (primask != 0U)
    {
        set_primask(0U);
    }
    if (tk_ref_sys(&rsys) >= E_OK && (rsys.sysstat & TSS_DDSP) != 0U &&
        (rsys.sysstat & (TSS_DINT | TSS_INDP)) == 0U)
    {
        (void)tk_ena_dsp();
    }
}

/*
 * When the kernel cannot block, wait on the hardware timer instead, so the
 * retry loop still runs at the back-off rate rather than flat out (the
 * 22 Nov flood). The core is no use to other tasks in that state anyway.
 */
#ifndef S4_SPIN_MS
LOCAL void s4_spin_ms(UW duration_ms)
{
    UW start = S4_HW_NOW_US();

    while ((UW)(S4_HW_NOW_US() - start) < duration_ms * 1000U)
    {
        /* busy-wait */
    }
}
#define S4_SPIN_MS(ms) s4_spin_ms(ms)
#endif

LOCAL void s4_delay(TMO duration_ms)
{
    ER ercd = tk_dly_tsk(duration_ms);

    if (ercd < E_OK)
    {
        handle_unblockable_core(ercd);
        if (ercd == E_CTX && duration_ms > 0)
        {
            S4_SPIN_MS((UW)duration_ms);
        }
    }
}

LOCAL void record_read_error(void)
{
    if (tk_loc_mtx(g_mtx_imu_data, S4_ERROR_MTX_TMO_MS) >= E_OK)
    {
        g_s4_imu_data.read_errors++;
        g_s4_imu_data.sensor_ok    = FALSE;
        g_s4_imu_data.motion_state = IMU_MOTION_FAULT;
        g_s4_imu_data.event        = IMU_EVENT_FAULT;
        tk_unl_mtx(g_mtx_imu_data);
    }
}

/* One failed init or read: mark the data invalid, report, recover, back off. */
LOCAL void imu_failed(const char * p_what, ER ercd)
{
    g_fail_streak++;
    record_read_error();
    report_error(p_what, ercd);
    if (ercd != E_CTX && (g_fail_streak % S4_RECOVER_EVERY_FAILS) == 0U)
    {
        (void)lsm303dlhc_recover_bus();
        g_bus_recoveries++;
    }
    s4_delay((TMO)s4_retry_delay_ms(g_fail_streak));
}

/* First good read after failures: say so once and restart the back-off. */
LOCAL void imu_read_ok(void)
{
    if (g_fail_streak > 0U)
    {
        LOG_INFO(TAG_S4_IMU,
                 "IMU recovered after %u failures (%u bus recoveries, %u "
                 "unreported)",
                 g_fail_streak, g_bus_recoveries, g_err_unreported);
        g_fail_streak = 0U;
    }
}

LOCAL BOOL update_calibration(const LSM303_SAMPLE * p_sample, W * p_baseline_x,
                              W * p_baseline_y, W * p_baseline_z)
{
    g_cal_state.sum_x += p_sample->ax_mg;
    g_cal_state.sum_y += p_sample->ay_mg;
    g_cal_state.sum_z += p_sample->az_mg;
    g_cal_state.count++;

    if (g_cal_state.count < S4_CALIBRATION_SAMPLES)
    {
        return FALSE;
    }

    *p_baseline_x = g_cal_state.sum_x / (W)g_cal_state.count;
    *p_baseline_y = g_cal_state.sum_y / (W)g_cal_state.count;
    *p_baseline_z = g_cal_state.sum_z / (W)g_cal_state.count;

    LOG_INFO(TAG_S4_IMU, "calibration complete base=(%d,%d,%d) mg",
             *p_baseline_x, *p_baseline_y, *p_baseline_z);

    return TRUE;
}

LOCAL void reset_lift_detector(void)
{
    UW idx;

    for (idx = 0U; idx < S4_LIFT_FILTER_SAMPLES; idx++)
    {
        g_lift_samples[idx] = 0;
    }
    g_lift_sum        = 0;
    g_lift_next       = 0U;
    g_lift_count      = 0U;
    g_lift_confirm    = 0U;
    g_b_lift_reported = FALSE;
}

/*
 * Lift (#53) = the averaged vertical acceleration stays away from the
 * calibrated baseline. Picking the robot up accelerates it upwards (Z rises)
 * and holding it tilted lowers Z; driving vibration averages out. Returns
 * TRUE on every sample while the condition has held for
 * S4_LIFT_CONFIRM_SAMPLES; *filtered_delta_mg is the averaged (clipped)
 * Z deviation from the baseline.
 */
LOCAL BOOL update_lift(W az_mg, W baseline_az_mg, W * p_filtered_delta_mg)
{
    W delta;
    W deviation = az_mg - baseline_az_mg;

    if (deviation > S4_LIFT_CLIP_MG)
    {
        deviation = S4_LIFT_CLIP_MG;
    }
    else if (deviation < -S4_LIFT_CLIP_MG)
    {
        deviation = -S4_LIFT_CLIP_MG;
    }
    else
    {
        /* No action needed in any other case. */
    }

    if (g_lift_count == S4_LIFT_FILTER_SAMPLES)
    {
        g_lift_sum -= g_lift_samples[g_lift_next];
    }
    else
    {
        g_lift_count++;
    }
    g_lift_samples[g_lift_next] = deviation;
    g_lift_sum += deviation;
    g_lift_next = (g_lift_next + 1U) % S4_LIFT_FILTER_SAMPLES;

    delta = g_lift_sum / (W)g_lift_count;
    if (p_filtered_delta_mg != NULL)
    {
        *p_filtered_delta_mg = delta;
    }

    /* Only a full window is averaged enough to judge. */
    if (g_lift_count == S4_LIFT_FILTER_SAMPLES &&
        abs_w(delta) >= S4_LIFT_DELTA_MG)
    {
        if (g_lift_confirm < S4_LIFT_CONFIRM_SAMPLES)
        {
            g_lift_confirm++;
        }
    }
    else
    {
        g_lift_confirm    = 0U;
        g_b_lift_reported = FALSE;
    }

    return (g_lift_confirm >= S4_LIFT_CONFIRM_SAMPLES) ? TRUE : FALSE;
}

LOCAL BOOL update_tilt(W lateral_mg, W forward_mg, BOOL b_hump_active)
{
    BOOL b_beyond_threshold;

    /*
     * X = lateral tilt.
     * A large sideways tilt remains dangerous even while crossing a ramp.
     *
     * Y = forward pitch.
     * A large Y deviation is expected while a ramp/hump is active,
     * so only treat forward pitch as tilt when not traversing a hump.
     */
    b_beyond_threshold =
        (abs_w(lateral_mg) >= S4_TILT_DELTA_MG) ||
        (!b_hump_active && abs_w(forward_mg) >= S4_TILT_DELTA_MG);

    if (b_beyond_threshold)
    {
        if (g_tilt_samples < S4_TILT_CONFIRM_SAMPLES)
        {
            g_tilt_samples++;
        }
    }
    else
    {
        g_tilt_samples    = 0U;
        g_b_tilt_reported = FALSE;
    }

    return (g_tilt_samples >= S4_TILT_CONFIRM_SAMPLES) ? TRUE : FALSE;
}

/* Multi-sample ramp pattern: up slope -> opposite slope -> flat again. */
LOCAL BOOL update_hump(W forward_mg, BOOL b_moving, UW distance_mm,
                       W * p_completed_peak)
{
    W abs_forward;

    abs_forward = abs_w(forward_mg);

    if (p_completed_peak != NULL)
    {
        *p_completed_peak = 0;
    }

    /*
     * After completion or timeout, require a sustained flat period
     * before another slope can start a measurement.
     */
    if (!g_b_hump_armed)
    {
        if (abs_forward <= S4_HUMP_SETTLE_MG)
        {
            if (g_hump_rearm_count < S4_HUMP_REARM_FLAT_SAMPLES)
            {
                g_hump_rearm_count++;
            }

            if (g_hump_rearm_count >= S4_HUMP_REARM_FLAT_SAMPLES)
            {
                g_b_hump_armed     = TRUE;
                g_hump_rearm_count = 0U;

                LOG_INFO(TAG_S4_IMU, "hump detector re-armed");
            }
        }
        else
        {
            g_hump_rearm_count = 0U;
        }

        return FALSE;
    }

    if (g_hump_state.phase != HUMP_IDLE)
    {
        g_hump_state.age_samples++;

        if (g_hump_state.age_samples > S4_HUMP_TIMEOUT_SAMPLES)
        {
            g_b_hump_armed     = FALSE;
            g_hump_rearm_count = 0U;
            reset_hump_detector();
            reset_height_tracker();
            LOG_WARN(TAG_S4_IMU, "hump timed out; waiting for level ground");
            return FALSE;
        }
    }

    switch (g_hump_state.phase)
    {
        case HUMP_IDLE:
        {
            INT current_sign = sign_w(forward_mg);

            /*
         * Climb since level ground (#27): the rise is counted from the last
         * level sample, not from the confirmation, because the ramp climbs
         * gently for ~0.5 s before the average reaches the entry level.
         * Reset on level ground or when the slope changes direction.
         */
            if (abs_forward <= S4_HUMP_SETTLE_MG ||
                current_sign != g_hump_climb_sign)
            {
                g_hump_entry_height_milli_mm = 0;
                g_hump_climb_sign =
                    (abs_forward <= S4_HUMP_SETTLE_MG) ? 0 : current_sign;
            }
            if (g_hump_climb_sign != 0)
            {
                g_hump_entry_height_milli_mm +=
                    (int64_t)distance_mm * (int64_t)abs_forward;
                if (g_hump_entry_height_milli_mm >
                    (int64_t)S4_MAX_RAMP_HEIGHT_MM * 1000)
                {
                    g_hump_entry_height_milli_mm =
                        (int64_t)S4_MAX_RAMP_HEIGHT_MM * 1000;
                }
            }

            /*
         * A hump may only begin when:
         *   1. the wheels are turning now (`moving`: this sample or the
         *      one before, so braking at a stop cannot start one), and
         *   2. the 200 ms average stays at the entry level, in one
         *      direction, for the confirmation count.
         */
            if (b_moving && abs_forward >= S4_HUMP_ENTRY_MG)
            {
                if (g_hump_state.entry_samples == 0U ||
                    current_sign != g_hump_state.entry_sign)
                {
                    g_hump_state.entry_sign    = current_sign;
                    g_hump_state.entry_samples = 1U;
                    g_hump_state.peak_mg       = abs_forward;
                }
                else
                {
                    g_hump_state.entry_samples++;

                    if (abs_forward > g_hump_state.peak_mg)
                    {
                        g_hump_state.peak_mg = abs_forward;
                    }
                }

                if (g_hump_state.entry_samples >= S4_HUMP_ENTRY_CONFIRM_SAMPLES)
                {
                    g_hump_state.phase          = HUMP_FIRST_SLOPE;
                    g_hump_state.first_sign     = g_hump_state.entry_sign;
                    g_hump_state.age_samples    = 0U;
                    g_hump_state.settle_samples = 0U;

                    g_hump_state.entry_sign    = 0;
                    g_hump_state.entry_samples = 0U;

                    LOG_INFO(TAG_S4_IMU,
                             "hump entry confirmed sign=%d peak=%dmg",
                             g_hump_state.first_sign, g_hump_state.peak_mg);
                }
            }
            else
            {
                /* The climb integral is kept: only level ground resets it. */
                g_hump_state.entry_sign    = 0;
                g_hump_state.entry_samples = 0U;
                g_hump_state.peak_mg       = 0;
            }

            break;
        }

        case HUMP_FIRST_SLOPE:

#if !S4_HUMP_TRACE
            LOG_INFO(TAG_S4_IMU,
                     "HUMP ACTIVE: forward=%d abs=%d dist=%u height=%u",
                     forward_mg, abs_forward, distance_mm, g_peak_height_mm);
#endif

            if (abs_forward > g_hump_state.peak_mg)
            {
                g_hump_state.peak_mg = abs_forward;
            }

            /*
        * Once a confirmed hump returns close to level ground,
        * begin checking that it remains flat.
        */
            if (abs_forward <= S4_HUMP_EXIT_FLAT_MG)
            {
                g_hump_state.phase          = HUMP_SETTLING;
                g_hump_state.settle_samples = 1U;
            }

            break;

        case HUMP_OPPOSITE_SLOPE:

            if (abs_forward > g_hump_state.peak_mg)
            {
                g_hump_state.peak_mg = abs_forward;
            }

            /*
         * Moving measurements are noisy, so use a slightly wider
         * "approximately flat" threshold for completion.
         */
            if (abs_forward <= S4_HUMP_EXIT_FLAT_MG)
            {
                g_hump_state.phase          = HUMP_SETTLING;
                g_hump_state.settle_samples = 1U;
            }

            break;

        case HUMP_SETTLING:

            if (abs_forward <= S4_HUMP_EXIT_FLAT_MG)
            {
                g_hump_state.settle_samples++;

                if (g_hump_state.settle_samples >= S4_HUMP_EXIT_FLAT_SAMPLES)
                {
                    if (p_completed_peak != NULL)
                    {
                        *p_completed_peak = g_hump_state.peak_mg;
                    }

                    g_b_hump_armed     = FALSE;
                    g_hump_rearm_count = 0U;

                    reset_hump_detector();

                    return TRUE;
                }
            }
            else
            {
                /*
            * Robot is not flat yet.
            * Continue tracking the hump.
            */
                g_hump_state.settle_samples = 0U;
                g_hump_state.phase          = HUMP_FIRST_SLOPE;
            }

            break;
        default:
            reset_hump_detector();
            reset_height_tracker();
            break;
    }

    return FALSE;
}

LOCAL void print_periodic_status(const LSM303_SAMPLE * p_sample, W baseline_x,
                                 W baseline_y, IMU_MOTION_STATE motion,
                                 W pitch_deg, UW ramp_height_mm,
                                 BOOL b_ramp_height_valid, UW sample_count)
{
    W delta_x;
    W delta_y;

    if ((sample_count % S4_PRINT_PERIOD_SAMPLES) != 0U)
    {
        return;
    }

    delta_x = p_sample->ax_mg - baseline_x;
    delta_y = p_sample->ay_mg - baseline_y;

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

             p_sample->ax_mg, p_sample->ay_mg, p_sample->az_mg,

             delta_x, delta_y,

             p_sample->mx_raw, p_sample->my_raw, p_sample->mz_raw,

             pitch_deg, ramp_height_mm, b_ramp_height_valid, motion);
}

EXPORT void imu_task(INT stacd, void * p_exinf)
{
    LSM303_SAMPLE sample;
    SYSTIM        cycle_start;
    SYSTIM        cycle_end;
    W             baseline_x = 0;
    W             baseline_y = 0;
    W             baseline_z = 0;
    W             delta_x;
    W             delta_y;
    W             completed_hump_peak;
    /* Moving average of the last five forward samples for hump detection. */
    W                hump_samples[S4_HUMP_FILTER_SAMPLES] = {0};
    W                hump_sum                             = 0;
    UW               hump_next                            = 0U;
    UW               hump_count                           = 0U;
    W                last_hump_peak                       = 0;
    W                pitch_deg                            = 0;
    W                lift_delta_mg                        = 0;
    UW               last_ramp_height                     = 0U;
    BOOL             b_last_ramp_height_valid             = FALSE;
    UW               distance_mm;
    UW               cycle_ms;
    UW               local_sample_count = 0U;
    BOOL             b_calibrated       = FALSE;
    BOOL             b_sensor_ok        = FALSE;
    BOOL             b_tilt_now;
    BOOL             b_hump_was_active;
    BOOL             b_hump_is_active;
    BOOL             b_hump_completed;
    BOOL             b_timing_overrun;
    IMU_MOTION_STATE motion;
    IMU_EVENT        event;
    ER               ercd;

    (void)stacd;
    (void)p_exinf;

    g_cal_state.sum_x       = 0;
    g_cal_state.sum_y       = 0;
    g_cal_state.sum_z       = 0;
    g_cal_state.count       = 0U;
    g_tilt_samples          = 0U;
    g_b_tilt_reported       = FALSE;
    g_b_hump_armed          = TRUE;
    g_hump_rearm_count      = 0U;
    g_movement_hold_samples = 0U;
    g_entry_moving_samples  = 0U;
    g_start_blank_samples   = 0U;
    reset_hump_detector();
    reset_height_tracker();
    reset_lift_detector();

    LOG_INFO(TAG_S4_IMU, "IMU task started, period=%d ms", S4_SAMPLE_PERIOD_MS);

    while (1)
    {
        tk_get_tim(&cycle_start);
        b_timing_overrun = FALSE;

        if (!b_sensor_ok)
        {
            ercd = lsm303dlhc_init();
            if (ercd < E_OK)
            {
                imu_failed("init", ercd);
                continue;
            }

            b_sensor_ok = TRUE;
            /* g_b_err_reported stays set: a device that re-inits and then fails
               again must not print once per retry (S4-U17). */
            b_calibrated            = FALSE;
            g_cal_state.sum_x       = 0;
            g_cal_state.sum_y       = 0;
            g_cal_state.sum_z       = 0;
            g_cal_state.count       = 0U;
            g_tilt_samples          = 0U;
            g_b_tilt_reported       = FALSE;
            g_b_hump_armed          = TRUE;
            g_hump_rearm_count      = 0U;
            g_movement_hold_samples = 0U;
            g_entry_moving_samples  = 0U;
            g_start_blank_samples   = 0U;
            hump_sum                = 0;
            hump_next               = 0U;
            hump_count              = 0U;
            reset_hump_detector();
            reset_height_tracker();
            reset_lift_detector();
        }

        ercd = lsm303dlhc_read(&sample);
        if (ercd < E_OK)
        {
            b_sensor_ok = FALSE;
            imu_failed("read", ercd);
            continue;
        }
        imu_read_ok();

        local_sample_count++;
        motion = IMU_MOTION_NORMAL;
        event  = IMU_EVENT_NONE;

        if (!b_calibrated)
        {
            b_calibrated = update_calibration(&sample, &baseline_x, &baseline_y,
                                              &baseline_z);

            publish_snapshot(&sample, baseline_x, baseline_y, baseline_z,
                             last_hump_peak, 0, last_ramp_height,
                             b_last_ramp_height_valid, IMU_MOTION_NORMAL,
                             IMU_EVENT_NONE, b_calibrated, TRUE, 0U, FALSE);
        }
        else
        {
            delta_x = sample.ax_mg - baseline_x;
            delta_y = sample.ay_mg - baseline_y;

            /*
            * Physical mounting confirmed from ramp testing:
            *
            * X = lateral / sideways movement
            * Y = forward / ramp slope
            *
            * Therefore pitch, hump detection and ramp-height estimation
            * use Y.
            */
            pitch_deg = estimate_pitch_deg(delta_y);

            distance_mm = subsystem4_take_distance_mm();
            if (g_start_blank_samples > 0U)
            {
                g_start_blank_samples--;
            }
            if (distance_mm > 0U)
            {
                if (g_movement_hold_samples == 0U)
                {
                    /* Starting from a standstill: blank hump starts. */
                    g_start_blank_samples = S4_HUMP_START_BLANK_SAMPLES;
                }
                g_movement_hold_samples = S4_MOVEMENT_HOLD_SAMPLES;
                g_entry_moving_samples  = S4_HUMP_ENTRY_MOVING_SAMPLES;
            }
            else
            {
                if (g_movement_hold_samples > 0U)
                {
                    g_movement_hold_samples--;
                }
                if (g_entry_moving_samples > 0U)
                {
                    g_entry_moving_samples--;
                }
            }

            completed_hump_peak = 0;

            /* Lift is the safety event (#53): highest classification
               priority. A hump in progress is abandoned. */
            if (update_lift(sample.az_mg, baseline_z, &lift_delta_mg))
            {
#if S4_HUMP_TRACE
                trace_record(local_sample_count, S4_TRACE_PH_LIFTED,
                             g_movement_hold_samples > 0U, delta_y,
                             lift_delta_mg, distance_mm);
                if (!g_b_lift_reported)
                {
                    trace_freeze((g_hump_state.phase != HUMP_IDLE)
                                     ? "lift (hump abandoned)"
                                     : "lift");
                }
#endif
                motion     = IMU_MOTION_LIFTED;
                hump_sum   = 0;
                hump_next  = 0U;
                hump_count = 0U;
                reset_hump_detector();
                reset_height_tracker();
                g_tilt_samples = 0U;

                if (!g_b_lift_reported)
                {
                    event             = IMU_EVENT_LIFT;
                    g_b_lift_reported = TRUE;
                    LOG_INFO(TAG_S4_IMU,
                             "EVENT LIFT z_avg-base=%dmg threshold=%dmg "
                             "acc=(%d,%d,%d) base_z=%d",
                             lift_delta_mg, S4_LIFT_DELTA_MG, sample.ax_mg,
                             sample.ay_mg, sample.az_mg, baseline_z);
                }
            }
            else
            {
                if (hump_count == S4_HUMP_FILTER_SAMPLES)
                {
                    hump_sum -= hump_samples[hump_next];
                }
                else
                {
                    hump_count++;
                }
                hump_samples[hump_next] = delta_y;
                hump_sum += delta_y;
                hump_next = (hump_next + 1U) % S4_HUMP_FILTER_SAMPLES;
                W hump_dy = hump_sum / (W)hump_count;

                b_hump_was_active = (g_hump_state.phase != HUMP_IDLE);
#if !S4_HUMP_TRACE
                if (!b_hump_was_active && g_b_hump_armed &&
                    abs_w(hump_dy) >= S4_HUMP_OPPOSITE_MG)
                {
                    LOG_INFO(TAG_S4_IMU,
                             "HUMP DEBUG: raw=%d filtered=%d moving=%d dist=%u "
                             "entry=%u",
                             delta_y, hump_dy, g_movement_hold_samples > 0U,
                             distance_mm, g_hump_state.entry_samples);
                }
#endif
                b_hump_completed = update_hump(
                    hump_dy,
                    (g_entry_moving_samples > 0U && g_start_blank_samples == 0U)
                        ? TRUE
                        : FALSE,
                    distance_mm, &completed_hump_peak);
                b_hump_is_active = (g_hump_state.phase != HUMP_IDLE);
#if S4_HUMP_TRACE
                /* Height is not updated yet for this sample; the next
                   record shows it. Good enough for a capture. */
                trace_record(local_sample_count,
                             g_b_hump_armed || b_hump_is_active ||
                                     b_hump_completed
                                 ? (UW)g_hump_state.phase
                                 : S4_TRACE_PH_UNARMED,
                             g_movement_hold_samples > 0U, delta_y, hump_dy,
                             distance_mm);
                trace_noise(
                    g_movement_hold_samples > 0U,
                    (!b_hump_was_active && !b_hump_is_active && g_b_hump_armed)
                        ? TRUE
                        : FALSE,
                    delta_y, hump_dy);
                if (b_hump_was_active && !b_hump_is_active && !b_hump_completed)
                {
                    trace_freeze("timeout");
                }
#endif

                if (!b_hump_was_active && b_hump_is_active)
                {
                    /*
                     * The entry-confirmation samples are already part of
                     * the uphill climb, so preserve their accumulated rise.
                     */
                    reset_height_tracker();

                    g_current_height_milli_mm = g_hump_entry_height_milli_mm;

                    g_current_height_mm =
                        calibrated_height_mm(g_current_height_milli_mm);

                    if (g_current_height_mm > 0)
                    {
                        g_peak_height_mm = (UW)g_current_height_mm;
                    }

                    if (g_hump_entry_height_milli_mm > 0)
                    {
                        g_b_height_distance_seen = TRUE;
                    }
                }

                if (b_hump_is_active)
                {
                    /*
                     * When the hump starts on this exact cycle, the current
                     * entry sample has already been included above.
                     */
                    if (b_hump_was_active)
                    {
                        update_height_estimate(distance_mm, hump_dy);
                    }

                    motion = IMU_MOTION_HUMP;
                }

                if (b_hump_completed && !g_b_height_distance_seen)
                {
                    /* No wheel distance during the whole hump: the robot
                       was not driving over anything (handled, or braking),
                       so this is not a hump (#27). */
                    LOG_INFO(TAG_S4_IMU,
                             "hump discarded: no wheel distance (peak=%d mg)",
                             completed_hump_peak);
#if S4_HUMP_TRACE
                    trace_freeze("discarded");
#endif
                }
                else if (b_hump_completed)
                {
                    motion         = IMU_MOTION_HUMP;
                    event          = IMU_EVENT_HUMP;
                    last_hump_peak = completed_hump_peak;
                    if (g_peak_height_mm > last_ramp_height)
                    {
                        last_ramp_height         = g_peak_height_mm;
                        b_last_ramp_height_valid = TRUE;
                    }
                    LOG_INFO(TAG_S4_IMU,
                             "EVENT HUMP height=%u mm highest=%u mm peak=%d mg",
                             g_peak_height_mm, last_ramp_height,
                             last_hump_peak);
#if S4_HUMP_TRACE
                    trace_freeze("event");
#endif
                }
                else
                {
                    /* A normal ramp should not be misclassified as severe tilt. */
                    b_tilt_now =
                        update_tilt(delta_x, delta_y, b_hump_is_active);

                    if (b_tilt_now)
                    {
                        W roll_deg = estimate_pitch_deg(delta_x);
                        motion     = IMU_MOTION_TILT;
                        if (!g_b_tilt_reported)
                        {
                            event             = IMU_EVENT_TILT;
                            g_b_tilt_reported = TRUE;
                            LOG_INFO(TAG_S4_IMU,
                                     "EVENT TILT roll=%d deg lateral=%d mg",
                                     roll_deg, delta_x);
                        }
                    }
                }
            }

            tk_get_tim(&cycle_end);
            cycle_ms         = cycle_end.lo - cycle_start.lo;
            b_timing_overrun = (cycle_ms > S4_SAMPLE_PERIOD_MS) ? TRUE : FALSE;

            publish_snapshot(&sample, baseline_x, baseline_y, baseline_z,
                             last_hump_peak, pitch_deg, last_ramp_height,
                             b_last_ramp_height_valid, motion, event, TRUE,
                             TRUE, cycle_ms, b_timing_overrun);

            print_periodic_status(&sample, baseline_x, baseline_y, motion,
                                  pitch_deg, last_ramp_height,
                                  b_last_ramp_height_valid, local_sample_count);
#if S4_HUMP_TRACE
            trace_dump_step();
#endif
        }

        wedge_followup();

        /* Maintain the configured interval between task cycles. */
        tk_get_tim(&cycle_end);
        cycle_ms = cycle_end.lo - cycle_start.lo;
#if S4_HUMP_TRACE
        g_trace_last_cycle_ms = cycle_ms;
#endif

        if (cycle_ms < S4_SAMPLE_PERIOD_MS)
        {
            s4_delay((TMO)(S4_SAMPLE_PERIOD_MS - cycle_ms));
        }
        else
        {
            /* Do not print every overrun: USB logging would worsen timing. */
            s4_delay(1);
        }
    }
}
