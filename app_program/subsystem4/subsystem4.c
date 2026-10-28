/*
 * subsystem4.c
 * Buddy 4 -- IMU & Terrain Monitoring (final requirements).
 */

#include "subsystem4.h"
#include "drv_lsm303dlhc.h"

EXPORT ID mtx_imu_data;
EXPORT IMU_DATA s4_imu_data;

/* Distance accumulated since the IMU task last consumed it. */
LOCAL UW pending_distance_mm;

LOCAL UB pack_status(const IMU_DATA *data)
{
    UB status = 0U;

    status |= ((UB)data->motion_state & S4_STATUS_MOTION_MASK);
    status |= (UB)((((UB)data->event) << S4_STATUS_EVENT_SHIFT)
                   & S4_STATUS_EVENT_MASK);

    if (data->calibrated) {
        status |= S4_STATUS_CALIBRATED;
    }

    if (!data->sensor_ok) {
        status |= S4_STATUS_IMU_FAULT;
    }

    return status;
}

EXPORT ER subsystem4_init(void)
{
    T_CMTX cmtx = {0};
    ER er;

    cmtx.mtxatr = TA_INHERIT;
    mtx_imu_data = tk_cre_mtx(&cmtx);
    if (mtx_imu_data < E_OK) {
        return mtx_imu_data;
    }

    pending_distance_mm = 0U;

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
    s4_imu_data.pitch_deg = 0;
    s4_imu_data.ramp_height_mm = 0U;
    s4_imu_data.motion_state = IMU_MOTION_FAULT;
    s4_imu_data.event = IMU_EVENT_NONE;
    s4_imu_data.calibrated = FALSE;
    s4_imu_data.sensor_ok = FALSE;
    s4_imu_data.sample_count = 0U;
    s4_imu_data.read_errors = 0U;
    s4_imu_data.last_cycle_ms = 0U;
    s4_imu_data.timing_overruns = 0U;

    /* Do not stop the whole firmware if the IMU is absent at boot. */
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

EXPORT ER subsystem4_get_packed(IMU_PACKED_MSG *out)
{
    UW height;
    W pitch;
    W peak;
    ER er;

    if (out == NULL) {
        return E_PAR;
    }

    er = tk_loc_mtx(mtx_imu_data, TMO_FEVR);
    if (er < E_OK) {
        return er;
    }

    out->status = pack_status(&s4_imu_data);

    height = s4_imu_data.ramp_height_mm;
    if (height > S4_MAX_RAMP_HEIGHT_MM) {
        height = S4_MAX_RAMP_HEIGHT_MM;
    }
    out->ramp_height_mm = (UB)height;

    pitch = s4_imu_data.pitch_deg;
    if (pitch < -90) {
        pitch = -90;
    } else if (pitch > 90) {
        pitch = 90;
    }
    out->pitch_encoded = (UB)(pitch + 90);

    peak = s4_imu_data.hump_peak_mg;
    if (peak < 0) {
        peak = 0;
    }
    peak /= 10;
    if (peak > 255) {
        peak = 255;
    }
    out->hump_peak_10mg = (UB)peak;

    tk_unl_mtx(mtx_imu_data);
    return E_OK;
}

EXPORT ER subsystem4_add_distance_mm(S4_DISTANCE_DELTA_MM delta_mm)
{
    ER er;

    er = tk_loc_mtx(mtx_imu_data, TMO_FEVR);
    if (er < E_OK) {
        return er;
    }

    pending_distance_mm += (UW)delta_mm;

    tk_unl_mtx(mtx_imu_data);
    return E_OK;
}

EXPORT UW subsystem4_take_distance_mm(void)
{
    UW distance;

    if (tk_loc_mtx(mtx_imu_data, TMO_FEVR) < E_OK) {
        return 0U;
    }

    distance = pending_distance_mm;
    pending_distance_mm = 0U;

    tk_unl_mtx(mtx_imu_data);
    return distance;
}
