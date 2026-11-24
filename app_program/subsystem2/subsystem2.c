/*
 * subsystem2.c
 * Buddy 2 -- creates Motion Control kernel objects and starts its task.
 */
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"

EXPORT ID g_mbf_motion_cmd;
EXPORT ID g_flg_motion_stop;
EXPORT ID g_mtx_motion_status;

/* The task owns writes to this value.  Other tasks can only copy it through
   subsystem2_get_status(), which holds g_mtx_motion_status. */
static MOTION_STATUS g_latest_status;

EXPORT ER subsystem2_init(void)
{
    T_CMBF cmbf = {0};
    T_CMTX cmtx = {0};
    T_CFLG cflg = {0};
    ER     ercd;

    /* Initialise hardware first; both drivers leave the vehicle stopped. */
    ercd = drv_motor_init();
    if (ercd < E_OK)
    {
        return ercd;
    }

    MOTOR_DIAGNOSTICS motor_diag;
    ercd = drv_motor_diagnostics(&motor_diag);
    LOG_INFO(TAG_S2_MOTOR, "PWM reset_done=%u counters L=%d R=%d check=%d",
             (UINT)(motor_diag.reset_done != 0u), motor_diag.left_counting,
             motor_diag.right_counting, ercd);
    if (ercd < E_OK)
    {
        drv_motor_stop();
        return ercd;
    }

    ercd = drv_encoder_init();
    if (ercd < E_OK)
    {
        drv_motor_stop();
        return ercd;
    }

    /* Buddy 2 receives copied MOTION_COMMAND structures here. */
    cmbf.mbfatr = TA_TFIFO;
    /* Kernel stores a length word alongside each copied message. */
    cmbf.bufsz =
        (INT)(S2_COMMAND_QUEUE_DEPTH * (sizeof(MOTION_COMMAND) + sizeof(UW)));
    cmbf.maxmsz = (INT)sizeof(MOTION_COMMAND);

    g_mbf_motion_cmd = tk_cre_mbf(&cmbf);

    if (g_mbf_motion_cmd < E_OK)
    {
        return g_mbf_motion_cmd;
    }

    cmtx.mtxatr = TA_INHERIT;

    g_mtx_motion_status = tk_cre_mtx(&cmtx);

    if (g_mtx_motion_status < E_OK)
    {
        return g_mtx_motion_status;
    }

    cflg.flgatr       = TA_TFIFO;
    g_flg_motion_stop = tk_cre_flg(&cflg);
    if (g_flg_motion_stop < E_OK)
    {
        return g_flg_motion_stop;
    }

    g_latest_status.state           = MOTION_STATE_INITIALISING;
    g_latest_status.active_command  = MOTION_CMD_NONE;
    g_latest_status.fault           = MOTION_FAULT_NONE;
    g_latest_status.motion_complete = false;

    g_latest_status.left_encoder_ticks  = 0;
    g_latest_status.right_encoder_ticks = 0;

    g_latest_status.left_speed_cm_s  = 0.0f;
    g_latest_status.right_speed_cm_s = 0.0f;
    g_latest_status.distance_cm      = 0.0f;

    g_latest_status.left_pwm_percent  = 0.0f;
    g_latest_status.right_pwm_percent = 0.0f;

    return E_OK;
}

EXPORT ER subsystem2_start(void)
{
    ID tid;

    tid = app_start_task(
#if S2_MOTOR_HW_TEST
        subsystem2_hw_test_task,
#else
        motion_task,
#endif
        S2_PRI_MOTION, TP_PRC2, "motion_task");

    if (tid < E_OK)
    {
        return tid;
    }

    return E_OK;
}

EXPORT ER subsystem2_send_command(const MOTION_COMMAND * p_command)
{
    if (p_command == 0)
    {
        return E_PAR;
    }

#if S2_MOTOR_HW_TEST
    return E_OBJ; /* Bench pulses bypass the closed-loop command consumer. */
#else
    if (p_command->type == MOTION_CMD_STOP)
    {
        return tk_set_flg(g_flg_motion_stop, 1u);
    }
#if APP_ROBOT_INTEGRATED || APP_TEST || S2_MOTION_DEMO
    return E_OBJ; /* Autonomous controller/test owns commands in these modes. */
#endif
#endif

    /*
     * Non-blocking send.
     *
     * If the message buffer is full, the caller receives an error instead
     * of being blocked indefinitely.
     */
    return tk_snd_mbf(g_mbf_motion_cmd, p_command, (INT)sizeof(MOTION_COMMAND),
                      TMO_POL);
}

EXPORT ER subsystem2_get_status(MOTION_STATUS * p_status)
{
    ER ercd;

    if (p_status == 0)
    {
        return E_PAR;
    }

    ercd = tk_loc_mtx(g_mtx_motion_status, TMO_FEVR);

    if (ercd < E_OK)
    {
        return ercd;
    }

    *p_status = g_latest_status;

    return tk_unl_mtx(g_mtx_motion_status);
}

/*
 * Only Motion Control should publish Buddy 2 status.
 *
 * Other subsystems should use subsystem2_get_status().
 */
void subsystem2_publish_status(const MOTION_STATUS * p_status)
{
    if (p_status == 0)
    {
        return;
    }

    if (tk_loc_mtx(g_mtx_motion_status, TMO_POL) == E_OK)
    {
        g_latest_status = *p_status;

        (void)tk_unl_mtx(g_mtx_motion_status);
    }
}
