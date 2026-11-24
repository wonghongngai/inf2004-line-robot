/*
 * subsystem4.c
 * Buddy 4 -- IMU & Terrain Monitoring (final requirements).
 */

#include "subsystem4.h"
#include "drv_lsm303dlhc.h"

EXPORT ID       g_mtx_imu_data;
EXPORT IMU_DATA g_s4_imu_data;

/* Distance accumulated since the IMU task last consumed it. */
LOCAL UW g_pending_distance_mm;

LOCAL UB pack_status(const IMU_DATA * p_data)
{
    UB status = 0U;

    status |= ((UB)p_data->motion_state & S4_STATUS_MOTION_MASK);
    status |= (UB)((((UB)p_data->event) << S4_STATUS_EVENT_SHIFT) &
                   S4_STATUS_EVENT_MASK);

    if (p_data->calibrated)
    {
        status |= S4_STATUS_CALIBRATED;
    }

    if (!p_data->sensor_ok)
    {
        status |= S4_STATUS_IMU_FAULT;
    }

    return status;
}

EXPORT ER subsystem4_init(void)
{
    T_CMTX cmtx = {0};
    ER     ercd;

    cmtx.mtxatr    = TA_INHERIT;
    g_mtx_imu_data = tk_cre_mtx(&cmtx);
    if (g_mtx_imu_data < E_OK)
    {
        return g_mtx_imu_data;
    }

    g_pending_distance_mm = 0U;

    g_s4_imu_data.ax_mg             = 0;
    g_s4_imu_data.ay_mg             = 0;
    g_s4_imu_data.az_mg             = 0;
    g_s4_imu_data.mx_raw            = 0;
    g_s4_imu_data.my_raw            = 0;
    g_s4_imu_data.mz_raw            = 0;
    g_s4_imu_data.baseline_ax_mg    = 0;
    g_s4_imu_data.baseline_ay_mg    = 0;
    g_s4_imu_data.baseline_az_mg    = 0;
    g_s4_imu_data.hump_peak_mg      = 0;
    g_s4_imu_data.pitch_deg         = 0;
    g_s4_imu_data.ramp_height_mm    = 0U;
    g_s4_imu_data.ramp_height_valid = FALSE;
    g_s4_imu_data.motion_state      = IMU_MOTION_FAULT;
    g_s4_imu_data.event             = IMU_EVENT_NONE;
    g_s4_imu_data.calibrated        = FALSE;
    g_s4_imu_data.sensor_ok         = FALSE;
    g_s4_imu_data.sample_count      = 0U;
    g_s4_imu_data.read_errors       = 0U;
    g_s4_imu_data.last_cycle_ms     = 0U;
    g_s4_imu_data.max_cycle_ms      = 0U;
    g_s4_imu_data.timing_overruns   = 0U;

    /* Do not stop the whole firmware if the IMU is absent at boot. */
    ercd = lsm303dlhc_init();
    if (ercd < E_OK)
    {
        LOG_WARN(TAG_S4_IMU, "IMU init deferred, er=%d", ercd);
    }
    else
    {
        g_s4_imu_data.sensor_ok = TRUE;
    }

    return E_OK;
}

EXPORT ER subsystem4_start(void)
{
    ID tid;

    tid = app_start_task(imu_task, S4_PRI_IMU, S4_IMU_PRC, "imu_task");
    return (tid < E_OK) ? tid : E_OK;
}

EXPORT ER subsystem4_get_latest(IMU_DATA * p_out)
{
    ER ercd;

    if (p_out == NULL)
    {
        return E_PAR;
    }

    ercd = tk_loc_mtx(g_mtx_imu_data, TMO_FEVR);
    if (ercd < E_OK)
    {
        return ercd;
    }

    *p_out = g_s4_imu_data;

    return tk_unl_mtx(g_mtx_imu_data);
}

EXPORT ER subsystem4_get_packed(IMU_PACKED_MSG * p_out)
{
    UW height;
    W  pitch;
    W  peak;
    ER ercd;

    if (p_out == NULL)
    {
        return E_PAR;
    }

    ercd = tk_loc_mtx(g_mtx_imu_data, TMO_FEVR);
    if (ercd < E_OK)
    {
        return ercd;
    }

    p_out->status = pack_status(&g_s4_imu_data);

    height = g_s4_imu_data.ramp_height_mm;
    if (height > S4_MAX_RAMP_HEIGHT_MM)
    {
        height = S4_MAX_RAMP_HEIGHT_MM;
    }
    p_out->ramp_height_mm = (UB)height;

    pitch = g_s4_imu_data.pitch_deg;
    if (pitch < -90)
    {
        pitch = -90;
    }
    else if (pitch > 90)
    {
        pitch = 90;
    }
    else
    {
        /* No action needed in any other case. */
    }
    p_out->pitch_encoded = (UB)(pitch + 90);

    peak = g_s4_imu_data.hump_peak_mg;
    if (peak < 0)
    {
        peak = 0;
    }
    peak /= 10;
    if (peak > 255)
    {
        peak = 255;
    }
    p_out->hump_peak_10mg = (UB)peak;

    tk_unl_mtx(g_mtx_imu_data);
    return E_OK;
}

EXPORT ER subsystem4_add_distance_mm(S4_DISTANCE_DELTA_MM delta_mm)
{
    ER ercd;

    ercd = tk_loc_mtx(g_mtx_imu_data, TMO_FEVR);
    if (ercd < E_OK)
    {
        return ercd;
    }

    g_pending_distance_mm += (UW)delta_mm;

    tk_unl_mtx(g_mtx_imu_data);
    return E_OK;
}

EXPORT UW subsystem4_take_distance_mm(void)
{
    UW distance;

    if (tk_loc_mtx(g_mtx_imu_data, TMO_FEVR) < E_OK)
    {
        return 0U;
    }

    distance              = g_pending_distance_mm;
    g_pending_distance_mm = 0U;

    tk_unl_mtx(g_mtx_imu_data);
    return distance;
}
