/*
 * subsystem4.c
 * Buddy 4 -- IMU & Terrain Monitoring (pre-change baseline).
 */

#include "subsystem4.h"
#include "drv_lsm303dlhc.h"

EXPORT ID mtx_imu_data;
EXPORT IMU_DATA s4_imu_data;

EXPORT ER subsystem4_init(void)
{
    T_CMTX cmtx = {0};
    ER er;

    /* Protect the latest Buddy 4 snapshot from partial reads. */
    cmtx.mtxatr = TA_INHERIT;
    mtx_imu_data = tk_cre_mtx(&cmtx);
    if (mtx_imu_data < E_OK) {
        return mtx_imu_data;
    }

    /* Start with a known public state. */
    s4_imu_data.ax_mg = 0;
    s4_imu_data.ay_mg = 0;
    s4_imu_data.az_mg = 0;
    s4_imu_data.mx_raw = 0;
    s4_imu_data.my_raw = 0;
    s4_imu_data.mz_raw = 0;
    s4_imu_data.baseline_ax_mg = 0;
    s4_imu_data.baseline_ay_mg = 0;
    s4_imu_data.baseline_az_mg = 0;
    s4_imu_data.hump_peak_mg = 0;
    s4_imu_data.motion_state = IMU_MOTION_FAULT;
    s4_imu_data.event = IMU_EVENT_NONE;
    s4_imu_data.calibrated = FALSE;
    s4_imu_data.sensor_ok = FALSE;
    s4_imu_data.sample_count = 0;
    s4_imu_data.read_errors = 0;
    s4_imu_data.last_cycle_ms = 0;

    /*
     * Try sensor setup now. If hardware is not connected yet, do not prevent
     * the rest of the firmware from starting; imu_task() will retry.
     */
    er = lsm303dlhc_init();
    if (er < E_OK) {
        tm_printf((UB *)"Buddy4: IMU init deferred, er=%d\n", er);
    } else {
        s4_imu_data.sensor_ok = TRUE;
    }

    return E_OK;
}

EXPORT ER subsystem4_start(void)
{
    ID tid;

    tid = app_start_task(imu_task, S4_PRI_IMU, 0, "imu_task");
    return (tid < E_OK) ? tid : E_OK;
}

EXPORT ER subsystem4_get_latest(IMU_DATA *out)
{
    ER er;

    if (out == NULL) {
        return E_PAR;
    }

    er = tk_loc_mtx(mtx_imu_data, TMO_FEVR);
    if (er < E_OK) {
        return er;
    }

    *out = s4_imu_data;

    return tk_unl_mtx(mtx_imu_data);
}
