/*
 * motion_control.h
 * Buddy 2 -- hardware-independent motion-control logic.
 *
 * This file intentionally uses ordinary C enums/structs/floats.  It does NOT
 * implement the Tribunal 1 minimum-bit data-representation change.
 */
#ifndef S2_MOTION_CONTROL_H
#define S2_MOTION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Commands Buddy 2 accepts.
 *
 * There is intentionally no backwards command.
 */
typedef enum
{
    MOTION_CMD_NONE = 0,

    MOTION_CMD_FORWARD_CM,

    MOTION_CMD_TURN_LEFT_DEG,

    MOTION_CMD_TURN_RIGHT_DEG,

    MOTION_CMD_STOP,

    MOTION_CMD_RESET_FAULT,

    /* Continuous forward travel; value = steering -1 (left)..+1 (right). */
    MOTION_CMD_FOLLOW_LINE

} MOTION_COMMAND_TYPE;

/*
 * value means:
 *
 * FORWARD_CM      -> centimetres
 * TURN_LEFT_DEG   -> degrees
 * TURN_RIGHT_DEG  -> degrees
 * STOP            -> unused
 * RESET_FAULT     -> unused
 */
typedef struct
{
    MOTION_COMMAND_TYPE type;

    float value;

} MOTION_COMMAND;

/*
 * Motion Control state machine.
 */
typedef enum
{
    MOTION_STATE_INITIALISING = 0,

    MOTION_STATE_IDLE,

    MOTION_STATE_MOVING_FORWARD,

    MOTION_STATE_TURNING_LEFT,

    MOTION_STATE_TURNING_RIGHT,

    MOTION_STATE_STOPPING,

    MOTION_STATE_FAULT

} MOTION_STATE;

/*
 * Faults currently detectable by Buddy 2.
 */
typedef enum
{
    MOTION_FAULT_NONE = 0,

    MOTION_FAULT_CONFIGURATION,

    MOTION_FAULT_LEFT_ENCODER_STALL,

    MOTION_FAULT_RIGHT_ENCODER_STALL

} MOTION_FAULT;

/*
 * Return result when a movement command is submitted.
 */
typedef enum
{
    MOTION_RESULT_OK = 0,

    MOTION_RESULT_BUSY = -1,

    MOTION_RESULT_INVALID_COMMAND = -2,

    MOTION_RESULT_FAULT_LATCHED = -3

} MOTION_RESULT;

/*
 * Motion configuration.
 *
 * These values are deliberately separate from the controller state so that
 * the physical robot can be calibrated later.
 */
typedef struct
{
    float left_ticks_per_cm;

    float right_ticks_per_cm;

    float left_turn_ticks_per_degree;

    float right_turn_ticks_per_degree;

    float base_pwm_percent;

    float minimum_effective_pwm_percent;

    float maximum_pwm_percent;

    float distance_tolerance_cm;

    float angle_tolerance_deg;

    uint32_t encoder_stall_timeout_ms;

    float kp;

    float ki;

    float kd;

    float integral_limit;

    float correction_limit;

    /*
     * Line following with all_test's tuned motor numbers (#8). Active when
     * follow_left_pwm_percent > 0; then FOLLOW_LINE drives these PWMs
     * directly, with no moving floor and no encoder-stall fault, as all_test
     * does. Zero keeps the original base_pwm/correction_limit behaviour.
     * Target 0 = cruise; 0 < |target| < 1 = soft (inner wheel at
     * follow_inner_pwm_percent); |target| >= 1 = hard (follow_hard_inner_...).
     * Distance moves, pivots and S5 manoeuvres always use base_pwm_percent.
     */
    float follow_left_pwm_percent; /* left cruise */

    float
        follow_right_pwm_percent; /* right cruise (the right motor is weaker) */

    float
        follow_boost_pwm_percent; /* added to the outer wheel while steering */

    float follow_inner_pwm_percent; /* inner wheel, soft correction */

    float follow_hard_inner_pwm_percent; /* inner wheel, hard correction */

    float
        follow_ramp_pwm_per_s; /* speed-up limit per wheel; slow-downs at once */

    /*
     * all_test's straight trim and barcode-scan slow-down for FOLLOW_LINE.
     * Trim is active when follow_straight_kp > 0, scan when follow_scan_scale
     * > 0 and motion_control_set_scanning() reports a scan in progress.
     */
    float follow_straight_kp; /* PWM points per cm of wheel difference */
    float follow_straight_max_trim;
    float follow_straight_trim_rate; /* PWM points per second */
    float follow_straight_deadband_cm;
    float follow_straight_ki;       /* PWM points per cm per second; 0 = off */
    float follow_straight_max_bias; /* cap on the learned bias */
    float follow_scan_scale;        /* 0..1 */
    float follow_scan_left_inner_pwm_percent;

    /*
     * all_test drive profiles (motion_control_set_profile()). Active when
     * move_left_pwm_percent > 0; otherwise every profile behaves as BASE.
     * Profile moves have no moving floor and no tolerance, ramp
     * move_ramp_pwm_per_s both ways, and may take over from FOLLOW_LINE
     * without a stop in between.
     */
    float move_left_pwm_percent;  /* FORWARD_CM, left wheel */
    float move_right_pwm_percent; /* FORWARD_CM, right wheel */
    float move_ramp_pwm_per_s;
    float
        junction_right_pivot_pwm_percent;  /* left wheel during a right pivot */
    float junction_left_pivot_pwm_percent; /* right wheel during a left pivot */
    float uturn_left_pwm_percent;          /* may be negative (reverse) */
    float uturn_right_pwm_percent;
    float avoid_heading_kp; /* PWM points cut per cm the leader is ahead */
    float avoid_pivot_pwm_percent;
    float
        avoid_left_turn_ticks_per_degree; /* left wheel during a right pivot */
    float
        avoid_right_turn_ticks_per_degree; /* right wheel during a left pivot */
    /* SPIN profile (S5 spin-in-place turns): both wheels counter-rotate at
       this PWM magnitude. 0 = no spin turns (they are rejected). */
    float avoid_spin_pwm_percent;

} MOTION_CONFIG;

/*
 * Which drive numbers the next FORWARD_CM / TURN_*_DEG command uses.
 *
 * BASE      base_pwm_percent, moving floor, tolerances (the original S2 moves)
 * JUNCTION  all_test's barcode junction: advance at move_*, pivots at
 *           junction_*_pivot with the S2 turn calibration
 * UTURN     TURN_LEFT_DEG counter-rotates at uturn_*; it runs until STOP
 *           (the caller ends it on the centre sensor, as all_test does)
 * AVOID     all_test's S5 moves: heading-held legs at move_*, pivots at
 *           avoid_pivot with avoid_*_turn_ticks_per_degree
 * SPIN      S5's spin-in-place turn (23 Nov): TURN_LEFT_DEG / TURN_RIGHT_DEG
 *           counter-rotate both wheels at avoid_spin_pwm_percent (left spin:
 *           left wheel back, right forward). Each wheel stops on its own
 *           once it has covered half its avoid pivot arc (the spin radius is
 *           half the pivot's), so the robot turns about its centre and ends
 *           where it began. FORWARD_CM is rejected in this profile.
 * REVERSE   S5's back-off (23 Nov): FORWARD_CM drives both wheels backwards
 *           at move_*, heading held as AVOID (the wheel that has gone
 *           further is slowed); turns are rejected in this profile.
 */
typedef enum
{
    MOTION_PROFILE_BASE = 0,
    MOTION_PROFILE_JUNCTION,
    MOTION_PROFILE_UTURN,
    MOTION_PROFILE_AVOID,
    MOTION_PROFILE_SPIN,
    MOTION_PROFILE_REVERSE
} MOTION_PROFILE;

/*
 * Information exposed by Buddy 2 to the rest of the system.
 */
typedef struct
{
    MOTION_STATE state;

    MOTION_COMMAND_TYPE active_command;

    MOTION_FAULT fault;

    bool motion_complete;

    int32_t left_encoder_ticks;

    int32_t right_encoder_ticks;

    float left_speed_cm_s;

    float right_speed_cm_s;

    float distance_cm;

    float left_pwm_percent;

    float right_pwm_percent;

} MOTION_STATUS;

/*
 * Internal Motion Control state.
 *
 * Other subsystems should not manipulate this directly.
 */
typedef struct
{
    MOTION_CONFIG cfg;

    MOTION_STATUS status;

    int32_t command_start_left_ticks;

    int32_t command_start_right_ticks;

    int32_t previous_left_ticks;

    int32_t previous_right_ticks;

    uint32_t previous_step_ms;

    uint32_t last_left_progress_ms;

    uint32_t last_right_progress_ms;

    float target_value;

    float pid_integral;

    float pid_previous_error;

    bool pid_has_previous_error;

    bool configuration_valid;

    /* Drive profiles: requested for the next command, and the running one. */
    MOTION_PROFILE next_profile;
    MOTION_PROFILE active_profile;
    /* PWM a profile move ended on, so a move or FOLLOW_LINE started just
       after it carries on from there instead of from rest. */
    float    carry_left_pwm, carry_right_pwm;
    uint32_t carry_ms;
    bool     carry_valid;
    /* FOLLOW_LINE straight trim and scan state. */
    bool    scanning;
    bool    straight_active;
    int32_t straight_left0, straight_right0;
    float   straight_trim;
    float   straight_bias; /* learned I term, kept across stretches */
    float   previous_follow_target;

} MOTION_CONTROLLER;

/*
 * Initialise Motion Control.
 *
 * Returns false if the calibration/configuration is invalid.
 */
bool motion_control_init(MOTION_CONTROLLER *   p_controller,
                         const MOTION_CONFIG * p_config, uint32_t now_ms,
                         int32_t left_ticks, int32_t right_ticks);

/*
 * Give Motion Control a new command.
 */
MOTION_RESULT motion_control_command(MOTION_CONTROLLER *    p_controller,
                                     const MOTION_COMMAND * p_command,
                                     uint32_t now_ms, int32_t left_ticks,
                                     int32_t right_ticks);

/*
 * Choose the drive profile for the next FORWARD_CM / TURN_*_DEG command.
 */
void motion_control_set_profile(MOTION_CONTROLLER * p_controller,
                                MOTION_PROFILE      profile);

/*
 * Tell FOLLOW_LINE whether a barcode is being scanned (slow-down).
 */
void motion_control_set_scanning(MOTION_CONTROLLER * p_controller,
                                 bool                b_scanning);

/*
 * One periodic Motion Control update.
 */
void motion_control_step(MOTION_CONTROLLER * p_controller, uint32_t now_ms,
                         int32_t left_ticks, int32_t right_ticks);

/*
 * Copy current Motion Control status.
 */
void motion_control_get_status(const MOTION_CONTROLLER * p_controller,
                               MOTION_STATUS *           p_status);

#endif /* S2_MOTION_CONTROL_H */
