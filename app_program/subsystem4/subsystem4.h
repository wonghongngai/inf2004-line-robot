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
 * - collision detection
 * - compact minimum-bit output using explicit masks/shifts
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

/* RTOS configuration. */
#define S4_PRI_IMU                  18
#define S4_SAMPLE_PERIOD_MS         20
#define S4_CALIBRATION_SAMPLES      50U
#define S4_PRINT_PERIOD_SAMPLES     25U

/*
 * Initial detection thresholds.
 * These MUST be tuned from the real robot logs.
 */
#define S4_TILT_DELTA_MG            350
#define S4_TILT_CONFIRM_SAMPLES     5U
#define S4_COLLISION_DELTA_MG       1200
#define S4_HUMP_ENTRY_MG            180
#define S4_HUMP_OPPOSITE_MG         120
#define S4_HUMP_SETTLE_MG           80
#define S4_HUMP_SETTLE_SAMPLES      5U
#define S4_HUMP_TIMEOUT_SAMPLES     75U

/*
 * Ramp-height output range.
 * 0..255 mm at 1 mm resolution fits in 8 bits.
 * Confirm the practice ramp is <= 255 mm high before final validation.
 */
#define S4_MAX_RAMP_HEIGHT_MM       255U

/* Five values need 3 bits. */
typedef enum {
    IMU_MOTION_NORMAL = 0,
    IMU_MOTION_HUMP = 1,
    IMU_MOTION_TILT = 2,
    IMU_MOTION_COLLISION = 3,
    IMU_MOTION_FAULT = 4
} IMU_MOTION_STATE;

/* Five values need 3 bits. */
typedef enum {
    IMU_EVENT_NONE = 0,
    IMU_EVENT_HUMP = 1,
    IMU_EVENT_TILT = 2,
    IMU_EVENT_COLLISION = 3,
    IMU_EVENT_FAULT = 4
} IMU_EVENT;

/*
 * Internal diagnostic state.
 * Raw sensor values remain full precision so detection accuracy is not lost.
 * Minimum-bit packing is applied only to the processed/shared output.
 */
typedef struct {
    W ax_mg;
    W ay_mg;
    W az_mg;

    W mx_raw;
    W my_raw;
    W mz_raw;

    W baseline_ax_mg;
    W baseline_ay_mg;
    W baseline_az_mg;

    W hump_peak_mg;
    W pitch_deg;
    UW ramp_height_mm;

    IMU_MOTION_STATE motion_state;
    IMU_EVENT event;

    BOOL calibrated;
    BOOL sensor_ok;

    UW sample_count;
    UW read_errors;
    UW last_cycle_ms;
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
typedef struct {
    UB status;
    UB ramp_height_mm;
    UB pitch_encoded;
    UB hump_peak_10mg;
} IMU_PACKED_MSG;

#define S4_STATUS_MOTION_MASK       0x07U
#define S4_STATUS_EVENT_MASK        0x38U
#define S4_STATUS_EVENT_SHIFT       3U
#define S4_STATUS_CALIBRATED        0x40U
#define S4_STATUS_IMU_FAULT         0x80U

/* Buddy 2 -> Buddy 4 distance input. One update is 0..255 mm (8 bits). */
typedef UB S4_DISTANCE_DELTA_MM;

IMPORT ID mtx_imu_data;
IMPORT IMU_DATA s4_imu_data;

IMPORT void imu_task(INT stacd, void *exinf);

EXPORT ER subsystem4_init(void);
EXPORT ER subsystem4_start(void);

/* Full internal/diagnostic snapshot. */
EXPORT ER subsystem4_get_latest(IMU_DATA *out);

/* Compact output for Vehicle Controller / telemetry. */
EXPORT ER subsystem4_get_packed(IMU_PACKED_MSG *out);

/*
 * Buddy 2 (or the Vehicle Controller) supplies distance travelled since the
 * previous update. Buddy 4 remains the owner of the height calculation.
 */
EXPORT ER subsystem4_add_distance_mm(S4_DISTANCE_DELTA_MM delta_mm);

/* Private-to-Buddy-4 consumption helper used by imu_task(). */
EXPORT UW subsystem4_take_distance_mm(void);

#endif /* SUBSYSTEM4_H */
