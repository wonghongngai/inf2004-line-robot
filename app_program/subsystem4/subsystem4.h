/*
 * subsystem4.h
 * Buddy 4 -- IMU & Terrain Monitoring (final requirements).
 *
 * Implements:
 * - 20 ms IMU sampling
 * - stationary calibration
 * - hump/ramp detection
 * - ramp-height estimation
 * - severe tilt detection
 * - lift detection (lifting the robot stops the motors; replaced collision
 *   detection, removed on the instructor's instructions 13 Nov, #53)
 * - compact minimum-bit output using explicit masks/shifts
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

/* RTOS configuration. */
#define S4_PRI_IMU 18
/*
 * Core for imu_task: core 0 (TP_PRC1), where the I2C0 interrupt lives
 * (dev_init_i2c(0) runs in knl_start_device() on core 0; the NVIC is per
 * core). Was 0 (any core) until 22 Nov 2026. With no affinity, imu_task ran
 * on core 1 next to motion_task, and every I2C transfer (~600/s) was woken
 * across cores by the core-0 I2C ISR. Both FATAL "core 2 cannot block"
 * reports on the robot (22 Nov, 9564a72) came from that core, in the I2C
 * read. A/B test of that path; it also keeps a stuck IMU core away from the
 * VC. Set to 0 to undo.
 */
#define S4_IMU_PRC TP_PRC1
#define S4_SAMPLE_PERIOD_MS \
    20 /* NFR4-1; was 10 from 18 to 20 Nov: cycles take ~12 ms (S4-U05) */
#define S4_CALIBRATION_SAMPLES  50U /* 1 s */
#define S4_PRINT_PERIOD_SAMPLES 50U /* 1 s */

/*
 * Initial detection thresholds.
 * These MUST be tuned from the real robot logs.
 */
#define S4_TILT_DELTA_MG        180
#define S4_TILT_CONFIRM_SAMPLES 5U /* 100 ms */
/*
 * Lift (#53): the average of the last S4_LIFT_FILTER_SAMPLES vertical (Z)
 * deviations from the calibrated baseline_az_mg is at least
 * S4_LIFT_DELTA_MG (either sign) for S4_LIFT_CONFIRM_SAMPLES samples in a
 * row. Single samples swing +-500 mg while driving and spike to +-1300 mg
 * (the old 1200 mg collision check tripped on them), so a raw sample is
 * never compared, and each deviation is clipped to S4_LIFT_CLIP_MG first:
 * a few spikes cannot carry the average, only a sustained offset can.
 * Placeholders until the S4-U12 bench test (lift 5x, humps 5x).
 */
#define S4_LIFT_DELTA_MG        200
#define S4_LIFT_CLIP_MG         400 /* 2 x threshold: half the window must agree */
#define S4_LIFT_FILTER_SAMPLES  10U /* 200 ms moving average */
#define S4_LIFT_CONFIRM_SAMPLES 3U  /* 60 ms */
/*
 * Hump detection (#27, tuned 23 Nov from the S4_TRACE captures of the 10 mm
 * one-side ramp). Every decision uses the 10-sample (200 ms) average of the
 * forward axis. On 16 s of recorded flat driving that average never stayed
 * >= 30 mg for more than 4 samples; on the ramp it stayed there for 59-89
 * samples, and never fell to <= 25 mg until the climb was over. The climb
 * takes 1.2-1.8 s at cruise, so the timeout allows 5 s.
 */
#define S4_HUMP_ENTRY_MG \
    30 /* 10-sample average, held for S4_HUMP_ENTRY_CONFIRM_SAMPLES */
#define S4_HUMP_OPPOSITE_MG     60   /* HUMP DEBUG print threshold only */
#define S4_HUMP_SETTLE_MG       20   /* level: re-arm and end of a hump */
#define S4_HUMP_TIMEOUT_SAMPLES 250U /* 5 s */

/*
 * Ramp-height output range.
 * 0..255 mm at 1 mm resolution fits in 8 bits.
 * Confirm the practice ramp is <= 255 mm high before final validation.
 */
#define S4_MAX_RAMP_HEIGHT_MM 255U

/* Five values need 3 bits. Code 3 was COLLISION until #53 and is now LIFTED. */
typedef enum
{
    IMU_MOTION_NORMAL = 0,
    IMU_MOTION_HUMP   = 1,
    IMU_MOTION_TILT   = 2,
    IMU_MOTION_LIFTED = 3, /* every sample while the lift check holds */
    IMU_MOTION_FAULT  = 4
} IMU_MOTION_STATE;

/* Five values need 3 bits. Code 3 was COLLISION until #53 and is now LIFT. */
typedef enum
{
    IMU_EVENT_NONE  = 0,
    IMU_EVENT_HUMP  = 1,
    IMU_EVENT_TILT  = 2,
    IMU_EVENT_LIFT  = 3, /* once per lift (lift_reported) */
    IMU_EVENT_FAULT = 4
} IMU_EVENT;

/*
 * Internal diagnostic state.
 * Raw sensor values remain full precision so detection accuracy is not lost.
 * Minimum-bit packing is applied only to the processed/shared output.
 */
typedef struct
{
    W ax_mg;
    W ay_mg;
    W az_mg;

    W mx_raw;
    W my_raw;
    W mz_raw;

    W baseline_ax_mg;
    W baseline_ay_mg;
    W baseline_az_mg;

    W    hump_peak_mg;
    W    pitch_deg;
    UW   ramp_height_mm;
    BOOL ramp_height_valid; /* distance was supplied for a completed hump */

    IMU_MOTION_STATE motion_state;
    IMU_EVENT        event;

    BOOL calibrated;
    BOOL sensor_ok;

    UW sample_count;
    UW read_errors;
    UW last_cycle_ms;
    UW max_cycle_ms; /* longest cycle since boot (calibration cycles report 0) */
    UW timing_overruns;
} IMU_DATA;

/*
 * Minimum-bit shared output (4 bytes total).
 *
 * status byte:
 *   bits 0..2 : motion state (3 bits)
 *   bits 3..5 : event        (3 bits)
 *   bit  6    : calibrated   (1 bit)
 *   bit  7    : IMU fault    (1 bit)
 *
 * ramp_height_mm:
 *   0..255 mm (8 bits)
 *
 * pitch_encoded:
 *   pitch + 90, so -90..+90 deg becomes 0..180 (8 bits)
 *
 * hump_peak_10mg:
 *   hump acceleration peak in 10 mg units (8 bits)
 */
typedef struct
{
    UB status;
    UB ramp_height_mm;
    UB pitch_encoded;
    UB hump_peak_10mg;
} IMU_PACKED_MSG;

#define S4_STATUS_MOTION_MASK 0x07U
#define S4_STATUS_EVENT_MASK  0x38U
#define S4_STATUS_EVENT_SHIFT 3U
#define S4_STATUS_CALIBRATED  0x40U
#define S4_STATUS_IMU_FAULT   0x80U

/* Buddy 2 -> Buddy 4 distance input. One update is 0..255 mm (8 bits). */
typedef UB S4_DISTANCE_DELTA_MM;

IMPORT ID       mtx_imu_data;
IMPORT IMU_DATA s4_imu_data;

IMPORT void imu_task(INT stacd, void * exinf);

EXPORT ER subsystem4_init(void);
EXPORT ER subsystem4_start(void);

/* Full internal/diagnostic snapshot. */
EXPORT ER subsystem4_get_latest(IMU_DATA * out);

/* Compact output for Vehicle Controller / telemetry. */
EXPORT ER subsystem4_get_packed(IMU_PACKED_MSG * out);

/*
 * Buddy 2 (or the Vehicle Controller) supplies distance travelled since the
 * previous update. Buddy 4 remains the owner of the height calculation.
 */
EXPORT ER subsystem4_add_distance_mm(S4_DISTANCE_DELTA_MM delta_mm);

/* Private-to-Buddy-4 consumption helper used by imu_task(). */
EXPORT UW subsystem4_take_distance_mm(void);

#endif /* SUBSYSTEM4_H */
