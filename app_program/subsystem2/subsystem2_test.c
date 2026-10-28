/*
 * subsystem2_test.c
 *
 * Buddy 2 on-target software tests.
 *
 * Build with:
 *
 *     make TEST=1
 *
 * These tests intentionally do NOT move the motors.
 *
 * They test the hardware-independent Motion Control logic using simulated
 * encoder counts.
 */
#include "subsystem2.h"
#include "../test_report.h"

#if APP_TEST

/*
 * Known-good TEST-ONLY configuration.
 *
 * These values are not physical robot calibration values.
 */
static MOTION_CONFIG test_config(void)
{
    MOTION_CONFIG cfg;

    cfg.left_ticks_per_cm =
        10.0f;

    cfg.right_ticks_per_cm =
        10.0f;

    cfg.left_turn_ticks_per_degree =
        2.0f;

    cfg.right_turn_ticks_per_degree =
        2.0f;

    cfg.base_pwm_percent =
        50.0f;

    cfg.minimum_effective_pwm_percent =
        10.0f;

    cfg.maximum_pwm_percent =
        100.0f;

    cfg.distance_tolerance_cm =
        0.5f;

    cfg.angle_tolerance_deg =
        1.0f;

    cfg.encoder_stall_timeout_ms =
        100U;

    cfg.kp =
        2.0f;

    cfg.ki =
        0.0f;

    cfg.kd =
        0.0f;

    cfg.integral_limit =
        50.0f;

    cfg.correction_limit =
        20.0f;

    return cfg;
}

EXPORT void subsystem2_test(void)
{
    T_RMBF rmbf;

    MOTION_CONTROLLER controller;

    MOTION_CONFIG cfg =
        test_config();

    MOTION_COMMAND command;

    MOTION_STATUS status;

    bool ok;

    TEST_INIT();

    /*
     * ------------------------------------------------------------
     * S2-U01
     *
     * Verify the Motion Control command interface is configured for
     * MOTION_COMMAND structures.
     * ------------------------------------------------------------
     */
    TEST_CHECK(
        "S2-U01",

        tk_ref_mbf(
            mbf_motion_cmd,
            &rmbf
        ) == E_OK &&

        rmbf.maxmsz ==
            (INT)sizeof(MOTION_COMMAND),

        "motion command message buffer has expected message size"
    );

    /*
     * ------------------------------------------------------------
     * S2-U02
     *
     * Simulated forward movement reaches the target distance.
     * ------------------------------------------------------------
     */
    ok =
        motion_control_init(
            &controller,
            &cfg,
            0U,
            0,
            0
        );

    command.type =
        MOTION_CMD_FORWARD_CM;

    command.value =
        10.0f;

    ok =
        ok &&

        (
            motion_control_command(
                &controller,
                &command,
                0U,
                0,
                0
            ) ==
            MOTION_RESULT_OK
        );

    /*
     * At 10 ticks/cm:
     *
     * 50 ticks = 5 cm.
     */
    motion_control_step(
        &controller,
        10U,
        50,
        50
    );

    /*
     * 100 ticks = 10 cm.
     */
    motion_control_step(
        &controller,
        20U,
        100,
        100
    );

    motion_control_get_status(
        &controller,
        &status
    );

    TEST_CHECK(
        "S2-U02",

        ok &&

        status.state ==
            MOTION_STATE_IDLE &&

        status.motion_complete &&

        status.left_pwm_percent ==
            0.0f &&

        status.right_pwm_percent ==
            0.0f,

        "forward target completes and motor outputs become zero"
    );

    /*
     * ------------------------------------------------------------
     * S2-U03
     *
     * STOP must interrupt active movement.
     * ------------------------------------------------------------
     */
    (void)motion_control_init(
        &controller,
        &cfg,
        0U,
        0,
        0
    );

    command.type =
        MOTION_CMD_FORWARD_CM;

    command.value =
        50.0f;

    (void)motion_control_command(
        &controller,
        &command,
        0U,
        0,
        0
    );

    motion_control_step(
        &controller,
        10U,
        10,
        10
    );

    command.type =
        MOTION_CMD_STOP;

    command.value =
        0.0f;

    (void)motion_control_command(
        &controller,
        &command,
        11U,
        10,
        10
    );

    motion_control_get_status(
        &controller,
        &status
    );

    TEST_CHECK(
        "S2-U03",

        status.state ==
            MOTION_STATE_STOPPING &&

        status.left_pwm_percent ==
            0.0f &&

        status.right_pwm_percent ==
            0.0f,

        "STOP immediately zeros both motor commands"
    );

    /*
     * ------------------------------------------------------------
     * S2-U04
     *
     * If Buddy 2 commands movement but the encoders do not change,
     * Motion Control must enter FAULT.
     * ------------------------------------------------------------
     */
    (void)motion_control_init(
        &controller,
        &cfg,
        0U,
        0,
        0
    );

    command.type =
        MOTION_CMD_FORWARD_CM;

    command.value =
        20.0f;

    (void)motion_control_command(
        &controller,
        &command,
        0U,
        0,
        0
    );

    /*
     * Timeout is 100 ms.
     *
     * Encoder values remain zero after 101 ms.
     */
    motion_control_step(
        &controller,
        101U,
        0,
        0
    );

    motion_control_get_status(
        &controller,
        &status
    );

    TEST_CHECK(
        "S2-U04",

        status.state ==
            MOTION_STATE_FAULT &&

        status.fault !=
            MOTION_FAULT_NONE &&

        status.left_pwm_percent ==
            0.0f &&

        status.right_pwm_percent ==
            0.0f,

        "encoder stall enters FAULT and stops both motors"
    );

    TEST_SUMMARY(
        "subsystem2"
    );
}

#endif /* APP_TEST */