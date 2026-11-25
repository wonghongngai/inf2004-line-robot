/*
 * motion_control.c
 * Buddy 2 -- Motion Control state machine, PID correction and fault handling.
 */
#include "motion_control.h"

/*
 * Return absolute value of a signed encoder count.
 */
static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/*
 * Keep a float between minimum and maximum.
 */
static float clamp_float(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

/*
 * Verify that Motion Control has usable calibration values.
 *
 * This prevents the robot from moving using guessed / invalid values.
 */
static bool config_is_valid(const MOTION_CONFIG * p_cfg)
{
    if (p_cfg == 0)
    {
        return false;
    }

    return

        (p_cfg->left_ticks_per_cm > 0.0f) &&

        (p_cfg->right_ticks_per_cm > 0.0f) &&

        (p_cfg->left_turn_ticks_per_degree > 0.0f) &&

        (p_cfg->right_turn_ticks_per_degree > 0.0f) &&

        (p_cfg->base_pwm_percent > 0.0f) &&

        (p_cfg->minimum_effective_pwm_percent >= 0.0f) &&

        (p_cfg->maximum_pwm_percent > 0.0f) &&

        (p_cfg->minimum_effective_pwm_percent <= p_cfg->base_pwm_percent) &&

        (p_cfg->base_pwm_percent <= p_cfg->maximum_pwm_percent) &&

        (p_cfg->maximum_pwm_percent <= 100.0f) &&

        (p_cfg->distance_tolerance_cm >= 0.0f) &&

        (p_cfg->angle_tolerance_deg >= 0.0f) &&

        (p_cfg->encoder_stall_timeout_ms > 0U) &&

        (p_cfg->integral_limit >= 0.0f) &&

        (p_cfg->correction_limit >= 0.0f) &&

        /* #8 follow-line motor numbers: 0 = legacy; otherwise every wheel
           output must lie in 0..maximum. */
        (p_cfg->follow_left_pwm_percent == 0.0f ||
         (p_cfg->follow_left_pwm_percent > 0.0f &&
          p_cfg->follow_right_pwm_percent > 0.0f &&
          p_cfg->follow_boost_pwm_percent >= 0.0f &&
          p_cfg->follow_inner_pwm_percent >= 0.0f &&
          p_cfg->follow_hard_inner_pwm_percent >= 0.0f &&
          p_cfg->follow_left_pwm_percent + p_cfg->follow_boost_pwm_percent <=
              p_cfg->maximum_pwm_percent &&
          p_cfg->follow_right_pwm_percent + p_cfg->follow_boost_pwm_percent <=
              p_cfg->maximum_pwm_percent &&
          p_cfg->follow_inner_pwm_percent <= p_cfg->maximum_pwm_percent &&
          p_cfg->follow_hard_inner_pwm_percent <=
              p_cfg->maximum_pwm_percent)) &&

        (p_cfg->follow_ramp_pwm_per_s >= 0.0f) &&

        /* Straight trim and scan slow-down: 0 = off. */
        (p_cfg->follow_straight_kp >= 0.0f) &&
        (p_cfg->follow_straight_max_trim >= 0.0f) &&
        (p_cfg->follow_straight_trim_rate >= 0.0f) &&
        (p_cfg->follow_straight_deadband_cm >= 0.0f) &&
        (p_cfg->follow_straight_ki >= 0.0f) &&
        (p_cfg->follow_straight_max_bias >= 0.0f) &&
        (p_cfg->follow_scan_scale >= 0.0f) &&
        (p_cfg->follow_scan_scale <= 1.0f) &&
        (p_cfg->follow_scan_left_inner_pwm_percent >= 0.0f) &&
        (p_cfg->follow_scan_left_inner_pwm_percent <=
         p_cfg->maximum_pwm_percent) &&
        (p_cfg->follow_left_pwm_percent + p_cfg->follow_straight_max_trim +
             p_cfg->follow_straight_max_bias <=
         p_cfg->maximum_pwm_percent) &&
        (p_cfg->follow_right_pwm_percent + p_cfg->follow_straight_max_trim +
             p_cfg->follow_straight_max_bias <=
         p_cfg->maximum_pwm_percent) &&

        /* all_test drive profiles: 0 = off; otherwise every output in range. */
        (p_cfg->move_left_pwm_percent == 0.0f ||
         (p_cfg->move_left_pwm_percent > 0.0f &&
          p_cfg->move_right_pwm_percent > 0.0f &&
          p_cfg->move_ramp_pwm_per_s >= 0.0f &&
          p_cfg->junction_right_pivot_pwm_percent > 0.0f &&
          p_cfg->junction_left_pivot_pwm_percent > 0.0f &&
          p_cfg->uturn_right_pwm_percent > 0.0f &&
          p_cfg->avoid_heading_kp >= 0.0f &&
          p_cfg->avoid_pivot_pwm_percent > 0.0f &&
          p_cfg->avoid_left_turn_ticks_per_degree > 0.0f &&
          p_cfg->avoid_right_turn_ticks_per_degree > 0.0f &&
          p_cfg->move_left_pwm_percent <= p_cfg->maximum_pwm_percent &&
          p_cfg->move_right_pwm_percent <= p_cfg->maximum_pwm_percent &&
          p_cfg->junction_right_pivot_pwm_percent <=
              p_cfg->maximum_pwm_percent &&
          p_cfg->junction_left_pivot_pwm_percent <=
              p_cfg->maximum_pwm_percent &&
          p_cfg->uturn_right_pwm_percent <= p_cfg->maximum_pwm_percent &&
          p_cfg->uturn_left_pwm_percent >= -p_cfg->maximum_pwm_percent &&
          p_cfg->uturn_left_pwm_percent <= p_cfg->maximum_pwm_percent &&
          p_cfg->avoid_pivot_pwm_percent <= p_cfg->maximum_pwm_percent &&
          p_cfg->avoid_spin_pwm_percent >= 0.0f &&
          p_cfg->avoid_spin_pwm_percent <= p_cfg->maximum_pwm_percent));
}

/* A profile move or FOLLOW_LINE starting within this long of a profile move
   ending carries on from its PWM (all_test never stops in between). */
#define MOTION_CARRY_MS 50U

/*
 * True when the running command uses an all_test drive profile.
 */
static bool profile_running(const MOTION_CONTROLLER * p_controller)
{
    return p_controller->active_profile != MOTION_PROFILE_BASE &&
           p_controller->cfg.move_left_pwm_percent > 0.0f;
}

/*
 * all_test's apply_follow_pwm(): move each wheel towards its target by at
 * most move_ramp_pwm_per_s, both ways; dt is capped at two periods so a late
 * step cannot become a sudden jump.
 */
static float ramp_move_pwm(const MOTION_CONTROLLER * p_controller,
                           float current, float target, float dt_s)
{
    float step;

    if (p_controller->cfg.move_ramp_pwm_per_s <= 0.0f)
    {
        return target;
    }
    if (dt_s > 0.020f)
    {
        dt_s = 0.020f;
    }
    step = p_controller->cfg.move_ramp_pwm_per_s * dt_s;
    if (target > current + step)
    {
        return current + step;
    }
    if (target < current - step)
    {
        return current - step;
    }
    return target;
}

/*
 * #8: soft start for line following, as all_test's apply_line_pwm(): a wheel
 * speeding up gains at most follow_ramp_pwm_per_s; slowing down is immediate.
 */
static float ramp_follow_pwm(const MOTION_CONTROLLER * p_controller,
                             float current, float target, float dt_s)
{
    float limited;

    if (p_controller->cfg.follow_ramp_pwm_per_s <= 0.0f || target <= current)
    {
        return target;
    }
    limited = current + p_controller->cfg.follow_ramp_pwm_per_s * dt_s;
    return limited < target ? limited : target;
}

/*
 * Set both requested motor outputs to zero.
 */
static void stop_outputs(MOTION_CONTROLLER * p_controller)
{
    p_controller->status.left_pwm_percent  = 0.0f;
    p_controller->status.right_pwm_percent = 0.0f;
}

/*
 * Clear the straight-line PID history.
 */
static void reset_pid(MOTION_CONTROLLER * p_controller)
{
    p_controller->pid_integral = 0.0f;

    p_controller->pid_previous_error = 0.0f;

    p_controller->pid_has_previous_error = false;
}

/*
 * Calculate straight-line PID correction.
 */
static float pid_update(MOTION_CONTROLLER * p_controller, float error,
                        float dt_s)
{
    float derivative = 0.0f;

    float output;

    if (dt_s <= 0.0f)
    {
        return 0.0f;
    }

    /*
     * Integral term.
     */
    p_controller->pid_integral += error * dt_s;

    p_controller->pid_integral = clamp_float(p_controller->pid_integral,
                                             -p_controller->cfg.integral_limit,
                                             p_controller->cfg.integral_limit);

    /*
     * Derivative is only available after there is a previous error.
     */
    if (p_controller->pid_has_previous_error)
    {
        derivative = (error - p_controller->pid_previous_error) / dt_s;
    }

    p_controller->pid_previous_error = error;

    p_controller->pid_has_previous_error = true;

    output = (p_controller->cfg.kp * error) +
             (p_controller->cfg.ki * p_controller->pid_integral) +
             (p_controller->cfg.kd * derivative);

    /*
     * Limit how strongly PID may modify the base PWM.
     */
    return clamp_float(output, -p_controller->cfg.correction_limit,
                       p_controller->cfg.correction_limit);
}

/*
 * Clamp a requested motor percentage.
 *
 * If movement is required, values below minimum_effective_pwm_percent
 * are raised to the minimum value which should physically move the wheel.
 */
static float moving_pwm(const MOTION_CONTROLLER * p_controller, float requested)
{
    if (requested <= 0.0f)
    {
        return 0.0f;
    }

    return clamp_float(requested,
                       p_controller->cfg.minimum_effective_pwm_percent,
                       p_controller->cfg.maximum_pwm_percent);
}

/*
 * Convert encoder ticks into centimetres.
 */
static float ticks_to_cm(int32_t delta_ticks, float ticks_per_cm)
{
    return (float)abs_i32(delta_ticks) / ticks_per_cm;
}

/*
 * Enter the fail-safe FAULT state.
 */
static void enter_fault(MOTION_CONTROLLER * p_controller, MOTION_FAULT fault)
{
    stop_outputs(p_controller);

    p_controller->carry_valid = false;

    p_controller->status.fault = fault;

    p_controller->status.state = MOTION_STATE_FAULT;

    p_controller->status.motion_complete = false;

    p_controller->status.active_command = MOTION_CMD_NONE;
}

/*
 * Common initialisation for Forward / Left / Right commands.
 */
static void begin_motion(MOTION_CONTROLLER * p_controller,
                         MOTION_COMMAND_TYPE type, float target_value,
                         MOTION_STATE state, uint32_t now_ms,
                         int32_t left_ticks, int32_t right_ticks)
{
    p_controller->target_value = target_value;

    /*
     * Save encoder counts at the beginning of this command.
     */
    p_controller->command_start_left_ticks = left_ticks;

    p_controller->command_start_right_ticks = right_ticks;

    p_controller->previous_left_ticks = left_ticks;

    p_controller->previous_right_ticks = right_ticks;

    p_controller->previous_step_ms = now_ms;

    p_controller->last_left_progress_ms = now_ms;

    p_controller->last_right_progress_ms = now_ms;

    p_controller->status.active_command = type;

    p_controller->status.state = state;

    p_controller->status.motion_complete = false;

    p_controller->status.fault = MOTION_FAULT_NONE;

    p_controller->status.distance_cm = 0.0f;

    p_controller->straight_active = false;

    p_controller->previous_follow_target = target_value;

    reset_pid(p_controller);
}

bool motion_control_init(MOTION_CONTROLLER *   p_controller,
                         const MOTION_CONFIG * p_config, uint32_t now_ms,
                         int32_t left_ticks, int32_t right_ticks)
{
    if ((p_controller == 0) || (p_config == 0))
    {
        return false;
    }

    p_controller->cfg = *p_config;

    p_controller->configuration_valid = config_is_valid(p_config);

    p_controller->target_value = 0.0f;

    p_controller->command_start_left_ticks = left_ticks;

    p_controller->command_start_right_ticks = right_ticks;

    p_controller->previous_left_ticks = left_ticks;

    p_controller->previous_right_ticks = right_ticks;

    p_controller->previous_step_ms = now_ms;

    p_controller->last_left_progress_ms = now_ms;

    p_controller->last_right_progress_ms = now_ms;

    reset_pid(p_controller);

    p_controller->next_profile           = MOTION_PROFILE_BASE;
    p_controller->active_profile         = MOTION_PROFILE_BASE;
    p_controller->carry_left_pwm         = 0.0f;
    p_controller->carry_right_pwm        = 0.0f;
    p_controller->carry_ms               = now_ms;
    p_controller->carry_valid            = false;
    p_controller->scanning               = false;
    p_controller->straight_active        = false;
    p_controller->straight_left0         = left_ticks;
    p_controller->straight_right0        = right_ticks;
    p_controller->straight_trim          = 0.0f;
    p_controller->straight_bias          = 0.0f;
    p_controller->previous_follow_target = 0.0f;

    p_controller->status.state = MOTION_STATE_INITIALISING;

    p_controller->status.active_command = MOTION_CMD_NONE;

    p_controller->status.fault = MOTION_FAULT_NONE;

    p_controller->status.motion_complete = false;

    p_controller->status.left_encoder_ticks = left_ticks;

    p_controller->status.right_encoder_ticks = right_ticks;

    p_controller->status.left_speed_cm_s = 0.0f;

    p_controller->status.right_speed_cm_s = 0.0f;

    p_controller->status.distance_cm = 0.0f;

    stop_outputs(p_controller);

    /*
     * Invalid calibration is a fault.
     *
     * The motors remain stopped.
     */
    if (!p_controller->configuration_valid)
    {
        enter_fault(p_controller, MOTION_FAULT_CONFIGURATION);

        return false;
    }

    p_controller->status.state = MOTION_STATE_IDLE;

    return true;
}

MOTION_RESULT motion_control_command(MOTION_CONTROLLER *    p_controller,
                                     const MOTION_COMMAND * p_command,
                                     uint32_t now_ms, int32_t left_ticks,
                                     int32_t right_ticks)
{
    if ((p_controller == 0) || (p_command == 0))
    {
        return MOTION_RESULT_INVALID_COMMAND;
    }

    /*
     * STOP is always accepted.
     *
     * However, STOP does NOT clear an existing FAULT.
     */
    if (p_command->type == MOTION_CMD_STOP)
    {
        stop_outputs(p_controller);

        p_controller->carry_valid = false;

        p_controller->straight_active = false;

        p_controller->status.active_command = MOTION_CMD_NONE;

        if (p_controller->status.state != MOTION_STATE_FAULT)
        {
            p_controller->status.state = MOTION_STATE_STOPPING;

            p_controller->status.motion_complete = true;
        }

        return MOTION_RESULT_OK;
    }

    /*
     * Explicit fault-reset command.
     */
    if (p_command->type == MOTION_CMD_RESET_FAULT)
    {
        /*
         * A configuration fault cannot be cleared while the
         * underlying calibration is still invalid.
         */
        if ((!p_controller->configuration_valid) ||
            (p_controller->status.state != MOTION_STATE_FAULT))
        {
            return MOTION_RESULT_FAULT_LATCHED;
        }

        p_controller->status.fault = MOTION_FAULT_NONE;

        p_controller->status.state = MOTION_STATE_IDLE;

        p_controller->status.active_command = MOTION_CMD_NONE;

        p_controller->status.motion_complete = false;

        p_controller->previous_left_ticks = left_ticks;

        p_controller->previous_right_ticks = right_ticks;

        p_controller->previous_step_ms = now_ms;

        p_controller->last_left_progress_ms = now_ms;

        p_controller->last_right_progress_ms = now_ms;

        reset_pid(p_controller);

        return MOTION_RESULT_OK;
    }

    /*
     * Normal motion commands cannot run while a fault is latched.
     */
    if (p_controller->status.state == MOTION_STATE_FAULT)
    {
        return MOTION_RESULT_FAULT_LATCHED;
    }

    /*
     * Prevent two movement commands from controlling the motors
     * simultaneously.
     */
    if (p_command->type == MOTION_CMD_FOLLOW_LINE &&
        p_command->value >= -1.0f && p_command->value <= 1.0f &&
        p_controller->status.active_command == MOTION_CMD_FOLLOW_LINE)
    {
        p_controller->target_value = p_command->value;
        return MOTION_RESULT_OK;
    }

    {
        bool b_profile_move =
            p_command->type != MOTION_CMD_FOLLOW_LINE &&
            p_controller->next_profile != MOTION_PROFILE_BASE &&
            p_controller->cfg.move_left_pwm_percent > 0.0f;
        bool b_from_follow =
            p_controller->status.active_command == MOTION_CMD_FOLLOW_LINE;

        /* all_test drives straight from line following onto a junction:
           a profile move may take over FOLLOW_LINE, keeping its PWM. */
        if (p_controller->status.state != MOTION_STATE_IDLE &&
            !(b_profile_move && b_from_follow))
        {
            return MOTION_RESULT_BUSY;
        }

        if (p_controller->next_profile == MOTION_PROFILE_UTURN &&
            b_profile_move && p_command->type != MOTION_CMD_TURN_LEFT_DEG)
        {
            return MOTION_RESULT_INVALID_COMMAND;
        }

        /* SPIN: turns only, and only if a spin PWM is configured. */
        if (p_controller->next_profile == MOTION_PROFILE_SPIN &&
            b_profile_move &&
            ((p_command->type != MOTION_CMD_TURN_LEFT_DEG &&
              p_command->type != MOTION_CMD_TURN_RIGHT_DEG) ||
             p_controller->cfg.avoid_spin_pwm_percent <= 0.0f))
        {
            return MOTION_RESULT_INVALID_COMMAND;
        }

        /* REVERSE: distance moves only. */
        if (p_controller->next_profile == MOTION_PROFILE_REVERSE &&
            b_profile_move && p_command->type != MOTION_CMD_FORWARD_CM)
        {
            return MOTION_RESULT_INVALID_COMMAND;
        }

        if (p_command->type == MOTION_CMD_FOLLOW_LINE ||
            p_command->type == MOTION_CMD_FORWARD_CM ||
            p_command->type == MOTION_CMD_TURN_LEFT_DEG ||
            p_command->type == MOTION_CMD_TURN_RIGHT_DEG)
        {
            bool b_numbers =
                p_command->type == MOTION_CMD_FOLLOW_LINE
                    ? p_controller->cfg.follow_left_pwm_percent > 0.0f
                    : b_profile_move;

            /* Start from where the last profile move left the wheels. */
            if (!b_from_follow && b_numbers && p_controller->carry_valid &&
                (uint32_t)(now_ms - p_controller->carry_ms) <= MOTION_CARRY_MS)
            {
                p_controller->status.left_pwm_percent =
                    p_controller->carry_left_pwm;
                p_controller->status.right_pwm_percent =
                    p_controller->carry_right_pwm;
            }
            p_controller->carry_valid    = false;
            p_controller->active_profile = b_profile_move
                                               ? p_controller->next_profile
                                               : MOTION_PROFILE_BASE;
        }
    }

    /*
     * Forward distance and turn angle must be positive.
     */
    if (p_command->type == MOTION_CMD_FOLLOW_LINE)
    {
        if (!(p_command->value >= -1.0f && p_command->value <= 1.0f))
        {
            return MOTION_RESULT_INVALID_COMMAND;
        }
        begin_motion(p_controller, p_command->type, p_command->value,
                     MOTION_STATE_MOVING_FORWARD, now_ms, left_ticks,
                     right_ticks);
        return MOTION_RESULT_OK;
    }

    if (!(p_command->value > 0.0f && p_command->value <= 100000.0f))
    {
        return MOTION_RESULT_INVALID_COMMAND;
    }

    switch (p_command->type)
    {
        case MOTION_CMD_FORWARD_CM:

            begin_motion(p_controller, p_command->type, p_command->value,
                         MOTION_STATE_MOVING_FORWARD, now_ms, left_ticks,
                         right_ticks);

            return MOTION_RESULT_OK;

        case MOTION_CMD_TURN_LEFT_DEG:

            begin_motion(p_controller, p_command->type, p_command->value,
                         MOTION_STATE_TURNING_LEFT, now_ms, left_ticks,
                         right_ticks);

            return MOTION_RESULT_OK;

        case MOTION_CMD_TURN_RIGHT_DEG:

            begin_motion(p_controller, p_command->type, p_command->value,
                         MOTION_STATE_TURNING_RIGHT, now_ms, left_ticks,
                         right_ticks);

            return MOTION_RESULT_OK;

        default: return MOTION_RESULT_INVALID_COMMAND;
    }
}

/*
 * Return true once the encoder has not made progress for the configured
 * amount of time.
 */
static bool timed_out(uint32_t now_ms, uint32_t last_progress_ms,
                      uint32_t timeout_ms)
{
    return (uint32_t)(now_ms - last_progress_ms) >= timeout_ms;
}

/*
 * Update encoder count, speed and progress timestamps.
 */
static void update_measurements(MOTION_CONTROLLER * p_controller,
                                uint32_t now_ms, int32_t left_ticks,
                                int32_t right_ticks)
{
    uint32_t dt_ms;

    int32_t left_delta;

    int32_t right_delta;

    dt_ms = (uint32_t)(now_ms - p_controller->previous_step_ms);

    left_delta = left_ticks - p_controller->previous_left_ticks;

    right_delta = right_ticks - p_controller->previous_right_ticks;

    p_controller->status.left_encoder_ticks = left_ticks;

    p_controller->status.right_encoder_ticks = right_ticks;

    /*
     * Calculate wheel speed.
     */
    if (dt_ms > 0U)
    {
        float dt_s = (float)dt_ms / 1000.0f;

        p_controller->status.left_speed_cm_s =
            ticks_to_cm(left_delta, p_controller->cfg.left_ticks_per_cm) / dt_s;

        p_controller->status.right_speed_cm_s =
            ticks_to_cm(right_delta, p_controller->cfg.right_ticks_per_cm) /
            dt_s;
    }

    /*
     * Record when each encoder last changed.
     */
    if (left_delta != 0)
    {
        p_controller->last_left_progress_ms = now_ms;
    }

    if (right_delta != 0)
    {
        p_controller->last_right_progress_ms = now_ms;
    }

    p_controller->previous_left_ticks = left_ticks;

    p_controller->previous_right_ticks = right_ticks;

    p_controller->previous_step_ms = now_ms;
}

/*
 * Finish a movement normally.
 */
static void complete_motion(MOTION_CONTROLLER * p_controller)
{
    /* A spin or a reverse ends with a wheel in reverse: never carry that
       into the next move (it starts from rest). */
    if (profile_running(p_controller) &&
        p_controller->active_profile != MOTION_PROFILE_SPIN &&
        p_controller->active_profile != MOTION_PROFILE_REVERSE)
    {
        p_controller->carry_left_pwm  = p_controller->status.left_pwm_percent;
        p_controller->carry_right_pwm = p_controller->status.right_pwm_percent;
        p_controller->carry_ms        = p_controller->previous_step_ms;
        p_controller->carry_valid     = true;
    }

    stop_outputs(p_controller);

    p_controller->status.active_command = MOTION_CMD_NONE;

    p_controller->status.state = MOTION_STATE_IDLE;

    p_controller->status.motion_complete = true;
}

/*
 * Forward-motion logic.
 *
 * Both wheel distances are calculated from the encoder feedback.
 *
 * PID compares left vs right wheel travel:
 *
 *      error = left distance - right distance
 *
 * If the left wheel moves too far, PID reduces left PWM and increases
 * right PWM.
 */
static void run_forward(MOTION_CONTROLLER * p_controller, uint32_t now_ms,
                        int32_t left_ticks, int32_t right_ticks, float dt_s)
{
    float left_cm;

    float right_cm;

    float average_cm;

    float remaining;

    float error;

    float correction;

    left_cm = ticks_to_cm(left_ticks - p_controller->command_start_left_ticks,
                          p_controller->cfg.left_ticks_per_cm);

    right_cm =
        ticks_to_cm(right_ticks - p_controller->command_start_right_ticks,
                    p_controller->cfg.right_ticks_per_cm);

    average_cm = (left_cm + right_cm) * 0.5f;

    remaining = p_controller->target_value - average_cm;

    p_controller->status.distance_cm = average_cm;

    /*
     * all_test's junction advance and S5 legs: no tolerance, no floor and,
     * as all_test, no stall fault (heading hold may stop the leading wheel);
     * the caller's move timeout covers a stuck robot.
     */
    if (p_controller->status.active_command == MOTION_CMD_FORWARD_CM &&
        profile_running(p_controller))
    {
        float l_target = p_controller->cfg.move_left_pwm_percent;
        float r_target = p_controller->cfg.move_right_pwm_percent;

        if (average_cm >= p_controller->target_value)
        {
            complete_motion(p_controller);
            return;
        }
        /* 22 Nov: the junction advance keeps the line-following PWM instead of
           ramping down to the move PWM, so the robot does not slow on the
           crossing (the follow PWM is higher than the move PWM). */
        if (p_controller->active_profile == MOTION_PROFILE_JUNCTION &&
            p_controller->cfg.follow_left_pwm_percent > 0.0f)
        {
            l_target = p_controller->cfg.follow_left_pwm_percent;
            r_target = p_controller->cfg.follow_right_pwm_percent;
        }
        if (p_controller->active_profile == MOTION_PROFILE_AVOID ||
            p_controller->active_profile == MOTION_PROFILE_REVERSE)
        {
            /* Distances are magnitudes, so this holds the heading backwards
               too: the wheel that has gone further is slowed. */
            float err =
                p_controller->cfg.avoid_heading_kp * (right_cm - left_cm);

            if (err > 0.0f)
            {
                r_target -= err; /* right wheel ahead: slow it */
            }
            else
            {
                l_target += err; /* left wheel ahead: slow it */
            }
            if (l_target < 0.0f)
            {
                l_target = 0.0f;
            }
            if (r_target < 0.0f)
            {
                r_target = 0.0f;
            }
        }
        if (p_controller->active_profile == MOTION_PROFILE_REVERSE)
        {
            l_target = -l_target; /* backwards */
            r_target = -r_target;
        }
        p_controller->status.left_pwm_percent =
            ramp_move_pwm(p_controller, p_controller->status.left_pwm_percent,
                          l_target, dt_s);
        p_controller->status.right_pwm_percent =
            ramp_move_pwm(p_controller, p_controller->status.right_pwm_percent,
                          r_target, dt_s);
        return;
    }

    /*
     * Target reached.
     */
    if (p_controller->status.active_command != MOTION_CMD_FOLLOW_LINE &&
        remaining <= p_controller->cfg.distance_tolerance_cm)
    {
        complete_motion(p_controller);

        return;
    }

    /*
     * all_test's line following has no stall stop: a wheel deliberately held
     * at 0-5 % would trip it. Distance moves and pivots keep the check (#8).
     */
    if (p_controller->status.active_command == MOTION_CMD_FOLLOW_LINE &&
        p_controller->cfg.follow_left_pwm_percent > 0.0f)
    {
        float l_target = p_controller->cfg.follow_left_pwm_percent;
        float r_target = p_controller->cfg.follow_right_pwm_percent;
        float target   = p_controller->target_value;
        bool  b_scan   = p_controller->scanning &&
                      p_controller->cfg.follow_scan_scale > 0.0f;

        if (target > 0.0f)
        { /* steer right: left wheel is outer */
            /* While scanning, no outer boost on the scanner-side (left)
               wheel: a +boost mid-frame sped GP28 up over the bars and
               garbled the widths (#23, 23 Nov: frames with 112 / 162 / 59 ms
               runs during 17/5 and 19/5 corrections). The right wheel still
               slows, so the robot still turns right. */
            if (!b_scan)
            {
                l_target += p_controller->cfg.follow_boost_pwm_percent;
            }
            r_target = target >= 1.0f
                           ? p_controller->cfg.follow_hard_inner_pwm_percent
                           : p_controller->cfg.follow_inner_pwm_percent;
        }
        else if (target < 0.0f)
        { /* steer left: right wheel is outer */
            r_target += p_controller->cfg.follow_boost_pwm_percent;
            /* While scanning, keep the scanner-side (left) wheel moving over
               the bars: no pivot on it (all_test, 19 Nov). */
            l_target =
                b_scan ? p_controller->cfg.follow_scan_left_inner_pwm_percent
                : target <= -1.0f
                    ? p_controller->cfg.follow_hard_inner_pwm_percent
                    : p_controller->cfg.follow_inner_pwm_percent;
        }
        else
        {
            /* No action needed in any other case. */
        }

        if (target == 0.0f && p_controller->cfg.follow_straight_kp > 0.0f)
        {
            /* all_test's white_straight_step(): boost the wheel that is
               behind since the stretch began; never cut one. */
            float l_cm, r_cm, err, trim_target, step;
            float dt_capped = dt_s > 0.020f ? 0.020f : dt_s;

            if (!p_controller->straight_active)
            {
                p_controller->straight_active = true;
                p_controller->straight_left0  = left_ticks;
                p_controller->straight_right0 = right_ticks;
                p_controller->straight_trim   = 0.0f;
            }
            l_cm = ticks_to_cm(left_ticks - p_controller->straight_left0,
                               p_controller->cfg.left_ticks_per_cm);
            r_cm = ticks_to_cm(right_ticks - p_controller->straight_right0,
                               p_controller->cfg.right_ticks_per_cm);
            err  = r_cm - l_cm; /* > 0: right ahead, boost left */
            if (err > p_controller->cfg.follow_straight_deadband_cm)
            {
                err -= p_controller->cfg.follow_straight_deadband_cm;
            }
            else if (err < -p_controller->cfg.follow_straight_deadband_cm)
            {
                err += p_controller->cfg.follow_straight_deadband_cm;
            }
            else
            {
                err = 0.0f;
            }
            trim_target =
                clamp_float(p_controller->cfg.follow_straight_kp * err,
                            -p_controller->cfg.follow_straight_max_trim,
                            p_controller->cfg.follow_straight_max_trim);
            step = p_controller->cfg.follow_straight_trim_rate * dt_capped;
            if (trim_target > p_controller->straight_trim + step)
            {
                p_controller->straight_trim += step;
            }
            else if (trim_target < p_controller->straight_trim - step)
            {
                p_controller->straight_trim -= step;
            }
            else
            {
                p_controller->straight_trim = trim_target;
            }
            /* I term: a steady mismatch shows up with the same sign on every
               stretch, so the bias it learns outlives the stretch. */
            if (p_controller->cfg.follow_straight_ki > 0.0f)
            {
                p_controller->straight_bias = clamp_float(
                    p_controller->straight_bias +
                        p_controller->cfg.follow_straight_ki * err * dt_capped,
                    -p_controller->cfg.follow_straight_max_bias,
                    p_controller->cfg.follow_straight_max_bias);
            }
            {
                float trim =
                    p_controller->straight_trim + p_controller->straight_bias;
                if (trim > 0.0f)
                {
                    l_target += trim;
                }
                else
                {
                    r_target -= trim;
                }
            }
        }
        else
        {
            p_controller->straight_active = false;
        }

        /* Correction over: the slowed inner wheel goes straight back to
           cruise, or its ramp keeps turning the robot past the line. */
        if (p_controller->previous_follow_target != 0.0f && target == 0.0f)
        {
            if (p_controller->status.left_pwm_percent <
                p_controller->cfg.follow_left_pwm_percent)
            {
                p_controller->status.left_pwm_percent =
                    p_controller->cfg.follow_left_pwm_percent;
            }
            if (p_controller->status.right_pwm_percent <
                p_controller->cfg.follow_right_pwm_percent)
            {
                p_controller->status.right_pwm_percent =
                    p_controller->cfg.follow_right_pwm_percent;
            }
        }
        p_controller->previous_follow_target = target;

        if (b_scan)
        {
            l_target *= p_controller->cfg.follow_scan_scale;
            r_target *= p_controller->cfg.follow_scan_scale;
        }
        p_controller->status.left_pwm_percent =
            ramp_follow_pwm(p_controller, p_controller->status.left_pwm_percent,
                            l_target, dt_s);
        p_controller->status.right_pwm_percent = ramp_follow_pwm(
            p_controller, p_controller->status.right_pwm_percent, r_target,
            dt_s);
        return;
    }

    /*
     * Check left encoder progress.
     */
    if (timed_out(now_ms, p_controller->last_left_progress_ms,
                  p_controller->cfg.encoder_stall_timeout_ms))
    {
        enter_fault(p_controller, MOTION_FAULT_LEFT_ENCODER_STALL);

        return;
    }

    /*
     * Check right encoder progress.
     */
    if (timed_out(now_ms, p_controller->last_right_progress_ms,
                  p_controller->cfg.encoder_stall_timeout_ms))
    {
        enter_fault(p_controller, MOTION_FAULT_RIGHT_ENCODER_STALL);

        return;
    }

    /*
     * Positive error means the left wheel has travelled farther.
     */
    error = left_cm - right_cm;

    correction = pid_update(p_controller, error, dt_s);

    if (p_controller->status.active_command == MOTION_CMD_FOLLOW_LINE)
    {
        /* Steering right speeds up the left wheel. PID remains responsible
           for straight travel; reset its distance history during curves. */
        if (p_controller->target_value != 0.0f)
        {
            correction = -p_controller->target_value *
                         p_controller->cfg.correction_limit;
            reset_pid(p_controller);
            p_controller->command_start_left_ticks  = left_ticks;
            p_controller->command_start_right_ticks = right_ticks;
        }
    }

    /*
     * Slow the wheel that is ahead and speed up the wheel that is behind.
     */
    p_controller->status.left_pwm_percent = moving_pwm(
        p_controller, p_controller->cfg.base_pwm_percent - correction);

    p_controller->status.right_pwm_percent = moving_pwm(
        p_controller, p_controller->cfg.base_pwm_percent + correction);
}

/*
 * Left pivot.
 *
 * Left wheel remains stopped while the right wheel moves forward.
 */
static void run_left_turn(MOTION_CONTROLLER * p_controller, uint32_t now_ms,
                          int32_t right_ticks, float dt_s)
{
    float turned_deg;

    bool b_profile = profile_running(p_controller);
    bool b_avoid =
        b_profile && p_controller->active_profile == MOTION_PROFILE_AVOID;

    turned_deg =
        (float)abs_i32(right_ticks - p_controller->command_start_right_ticks) /
        (b_avoid ? p_controller->cfg.avoid_right_turn_ticks_per_degree
                 : p_controller->cfg.right_turn_ticks_per_degree);

    /*
     * Target angle reached (profile pivots: no tolerance, as all_test).
     */
    if ((p_controller->target_value - turned_deg) <=
        (b_profile ? 0.0f : p_controller->cfg.angle_tolerance_deg))
    {
        complete_motion(p_controller);

        return;
    }

    /*
     * The active wheel should produce encoder movement.
     */
    if (timed_out(now_ms, p_controller->last_right_progress_ms,
                  p_controller->cfg.encoder_stall_timeout_ms))
    {
        enter_fault(p_controller, MOTION_FAULT_RIGHT_ENCODER_STALL);

        return;
    }

    p_controller->status.left_pwm_percent = 0.0f;

    p_controller->status.right_pwm_percent =
        !b_profile
            ? p_controller->cfg.base_pwm_percent
            : ramp_move_pwm(
                  p_controller, p_controller->status.right_pwm_percent,
                  b_avoid ? p_controller->cfg.avoid_pivot_pwm_percent
                          : p_controller->cfg.junction_left_pivot_pwm_percent,
                  dt_s);
}

/*
 * Right pivot.
 *
 * Right wheel remains stopped while left wheel moves forward.
 */
static void run_right_turn(MOTION_CONTROLLER * p_controller, uint32_t now_ms,
                           int32_t left_ticks, float dt_s)
{
    float turned_deg;

    bool b_profile = profile_running(p_controller);
    bool b_avoid =
        b_profile && p_controller->active_profile == MOTION_PROFILE_AVOID;

    turned_deg =
        (float)abs_i32(left_ticks - p_controller->command_start_left_ticks) /
        (b_avoid ? p_controller->cfg.avoid_left_turn_ticks_per_degree
                 : p_controller->cfg.left_turn_ticks_per_degree);

    /*
     * Target angle reached (profile pivots: no tolerance, as all_test).
     */
    if ((p_controller->target_value - turned_deg) <=
        (b_profile ? 0.0f : p_controller->cfg.angle_tolerance_deg))
    {
        complete_motion(p_controller);

        return;
    }

    /*
     * Check active left wheel.
     */
    if (timed_out(now_ms, p_controller->last_left_progress_ms,
                  p_controller->cfg.encoder_stall_timeout_ms))
    {
        enter_fault(p_controller, MOTION_FAULT_LEFT_ENCODER_STALL);

        return;
    }

    p_controller->status.left_pwm_percent =
        !b_profile
            ? p_controller->cfg.base_pwm_percent
            : ramp_move_pwm(
                  p_controller, p_controller->status.left_pwm_percent,
                  b_avoid ? p_controller->cfg.avoid_pivot_pwm_percent
                          : p_controller->cfg.junction_right_pivot_pwm_percent,
                  dt_s);

    p_controller->status.right_pwm_percent = 0.0f;
}

/*
 * all_test's U-turn: both wheels counter-rotate until the caller sends STOP
 * (it watches the right wheel's ticks and the centre sensor). The right
 * wheel must keep making progress.
 */
static void run_uturn(MOTION_CONTROLLER * p_controller, uint32_t now_ms,
                      float dt_s)
{
    if (timed_out(now_ms, p_controller->last_right_progress_ms,
                  p_controller->cfg.encoder_stall_timeout_ms))
    {
        enter_fault(p_controller, MOTION_FAULT_RIGHT_ENCODER_STALL);

        return;
    }

    p_controller->status.left_pwm_percent =
        ramp_move_pwm(p_controller, p_controller->status.left_pwm_percent,
                      p_controller->cfg.uturn_left_pwm_percent, dt_s);

    p_controller->status.right_pwm_percent =
        ramp_move_pwm(p_controller, p_controller->status.right_pwm_percent,
                      p_controller->cfg.uturn_right_pwm_percent, dt_s);
}

/*
 * SPIN profile: turn on the spot. Both wheels counter-rotate (left spin:
 * left wheel back, right forward; right spin the mirror). A spin's wheel
 * arc is half a one-wheel pivot's (radius half the track), so each wheel
 * needs half the avoid pivot ticks per degree. Each wheel stops by itself
 * once it has covered its share; the turn is complete when both have, so
 * a faster wheel cannot over-turn the robot. A wheel still driving must
 * keep making progress.
 */
static void run_spin(MOTION_CONTROLLER * p_controller, uint32_t now_ms,
                     int32_t left_ticks, int32_t right_ticks, bool b_left_spin,
                     float dt_s)
{
    float left_target = p_controller->target_value * 0.5f *
                        p_controller->cfg.avoid_left_turn_ticks_per_degree;
    float right_target = p_controller->target_value * 0.5f *
                         p_controller->cfg.avoid_right_turn_ticks_per_degree;
    bool b_left_done =
        (float)abs_i32(left_ticks - p_controller->command_start_left_ticks) >=
        left_target;
    bool b_right_done =
        (float)abs_i32(right_ticks - p_controller->command_start_right_ticks) >=
        right_target;
    float pwm = p_controller->cfg.avoid_spin_pwm_percent;

    if (b_left_done && b_right_done)
    {
        complete_motion(p_controller);
        return;
    }

    if ((!b_left_done && timed_out(now_ms, p_controller->last_left_progress_ms,
                                   p_controller->cfg.encoder_stall_timeout_ms)))
    {
        enter_fault(p_controller, MOTION_FAULT_LEFT_ENCODER_STALL);
        return;
    }
    if ((!b_right_done &&
         timed_out(now_ms, p_controller->last_right_progress_ms,
                   p_controller->cfg.encoder_stall_timeout_ms)))
    {
        enter_fault(p_controller, MOTION_FAULT_RIGHT_ENCODER_STALL);
        return;
    }

    p_controller->status.left_pwm_percent =
        b_left_done
            ? 0.0f
            : ramp_move_pwm(p_controller, p_controller->status.left_pwm_percent,
                            b_left_spin ? -pwm : pwm, dt_s);
    p_controller->status.right_pwm_percent =
        b_right_done ? 0.0f
                     : ramp_move_pwm(p_controller,
                                     p_controller->status.right_pwm_percent,
                                     b_left_spin ? pwm : -pwm, dt_s);
}

/*
 * One periodic Motion Control iteration.
 */
void motion_control_step(MOTION_CONTROLLER * p_controller, uint32_t now_ms,
                         int32_t left_ticks, int32_t right_ticks)
{
    uint32_t dt_ms;

    float dt_s;

    if (p_controller == 0)
    {
        return;
    }

    dt_ms = (uint32_t)(now_ms - p_controller->previous_step_ms);

    /*
     * First update encoder/speed measurements.
     */
    if (!p_controller->configuration_valid)
    {
        p_controller->status.left_encoder_ticks  = left_ticks;
        p_controller->status.right_encoder_ticks = right_ticks;
        stop_outputs(p_controller);
        return;
    }

    update_measurements(p_controller, now_ms, left_ticks, right_ticks);

    dt_s = (dt_ms > 0U) ? ((float)dt_ms / 1000.0f) : 0.0f;

    switch (p_controller->status.state)
    {
        case MOTION_STATE_MOVING_FORWARD:

            run_forward(p_controller, now_ms, left_ticks, right_ticks, dt_s);

            break;

        case MOTION_STATE_TURNING_LEFT:

            if (profile_running(p_controller) &&
                p_controller->active_profile == MOTION_PROFILE_UTURN)
            {
                run_uturn(p_controller, now_ms, dt_s);

                break;
            }

            if (profile_running(p_controller) &&
                p_controller->active_profile == MOTION_PROFILE_SPIN)
            {
                run_spin(p_controller, now_ms, left_ticks, right_ticks, true,
                         dt_s);

                break;
            }

            run_left_turn(p_controller, now_ms, right_ticks, dt_s);

            break;

        case MOTION_STATE_TURNING_RIGHT:

            if (profile_running(p_controller) &&
                p_controller->active_profile == MOTION_PROFILE_SPIN)
            {
                run_spin(p_controller, now_ms, left_ticks, right_ticks, false,
                         dt_s);

                break;
            }

            run_right_turn(p_controller, now_ms, left_ticks, dt_s);

            break;

        case MOTION_STATE_STOPPING:

            stop_outputs(p_controller);

            p_controller->status.state = MOTION_STATE_IDLE;

            break;

        case MOTION_STATE_FAULT:

        case MOTION_STATE_IDLE:

        case MOTION_STATE_INITIALISING:

        default: stop_outputs(p_controller); break;
    }
}

void motion_control_set_profile(MOTION_CONTROLLER * p_controller,
                                MOTION_PROFILE      profile)
{
    if (p_controller != 0)
    {
        p_controller->next_profile = profile;
    }
}

void motion_control_set_scanning(MOTION_CONTROLLER * p_controller,
                                 bool                b_scanning)
{
    if (p_controller != 0)
    {
        p_controller->scanning = b_scanning;
    }
}

/*
 * Return a copy of the current status.
 */
void motion_control_get_status(const MOTION_CONTROLLER * p_controller,
                               MOTION_STATUS *           p_status)
{
    if ((p_controller != 0) && (p_status != 0))
    {
        *p_status = p_controller->status;
    }
}
