/*
 * subsystem4.h
 * Buddy 4 -- IMU & Terrain Monitoring (pre-change baseline).
 *
 * This version intentionally uses normal C data types. It does NOT include
 * the later minimum-bit packing requirement.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

/* Buddy 4 priority band from TEAM.md: 18-19. */
#define S4_PRI_IMU                 18

/* Week 6 sampling requirement: one IMU sample every 20 ms (50 Hz). */
#define S4_SAMPLE_PERIOD_MS        20

/* Calibration uses 1 second of stationary samples at 50 Hz. */
#define S4_CALIBRATION_SAMPLES     50U

/* Avoid printing every 20 ms because console output disturbs task timing. */
#define S4_PRINT_PERIOD_SAMPLES    25U

/*
 * Initial detection thresholds.
 * These are STARTING VALUES ONLY and must be replaced/tuned using actual
 * robot logs from normal driving, hump crossing, tilt and controlled impact.
 */
#define S4_TILT_DELTA_MG           350
#define S4_TILT_CONFIRM_SAMPLES    5U
#define S4_COLLISION_DELTA_MG      1200
#define S4_HUMP_ENTRY_MG           180
#define S4_HUMP_OPPOSITE_MG        120
#define S4_HUMP_SETTLE_MG          80
#define S4_HUMP_SETTLE_SAMPLES     5U
#define S4_HUMP_TIMEOUT_SAMPLES    75U

typedef enum {
    IMU_MOTION_NORMAL = 0,
    IMU_MOTION_HUMP,
    IMU_MOTION_TILT,
    IMU_MOTION_COLLISION,
    IMU_MOTION_FAULT
} IMU_MOTION_STATE;

typedef enum {
    IMU_EVENT_NONE = 0,
    IMU_EVENT_HUMP,
    IMU_EVENT_TILT,
    IMU_EVENT_COLLISION,
    IMU_EVENT_FAULT
} IMU_EVENT;

/*
 * Latest processed Buddy 4 data.
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

    IMU_MOTION_STATE motion_state;
    IMU_EVENT event;

    BOOL calibrated;
    BOOL sensor_ok;

    UW sample_count;
    UW read_errors;
    UW last_cycle_ms;
} IMU_DATA;

/* Private Buddy 4 kernel object used to protect the latest snapshot. */
IMPORT ID mtx_imu_data;

/* Latest processed data. Read it through subsystem4_get_latest(). */
IMPORT IMU_DATA s4_imu_data;

/* Task entry point. */
IMPORT void imu_task(INT stacd, void *exinf);

/* Standard subsystem lifecycle used by app_main.c. */
EXPORT ER subsystem4_init(void);
EXPORT ER subsystem4_start(void);

/* Safe read-only interface for tests / later Vehicle Controller integration. */
EXPORT ER subsystem4_get_latest(IMU_DATA *out);

#endif /* SUBSYSTEM4_H */
