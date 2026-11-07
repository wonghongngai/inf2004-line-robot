#include "all_buddies_test.h"

#if APP_ALL_TEST
#include "drv_motor.h"
#include "drv_encoder.h"
#include "subsystem2.h"
#include "wheel_odometry.h"
#include "line_follow_output.h"
#include "../subsystem1/subsystem1.h"
#include "../subsystem3/line_service.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem5/s5_executor.h"

enum { WAITING, SETTLING, RUNNING, PAUSED, STOPPED, LINE_WAIT };
enum { TURN_NONE, TURN_ADVANCE, TURN_PIVOT };
#define ALL_START_MS       5000u
#define ALL_READY_MS       15000u
#define ALL_IMU_STALE_MS   100u
#define ALL_LINE_READY_MS  500u
#define ALL_LINE_REACQUIRE_MS 100u
#define ALL_BASE_PWM       12.5f
#define ALL_INNER_PWM      6.25f
#define ALL_RIGHT_BASE_PWM 15.0f
#define ALL_RIGHT_TURN_LEFT_PWM 12.5f
#define ALL_WHITE_CLEAR_MS 100u

typedef struct {
    unsigned phase, terminal_state, barcode;
    uint32_t boot, started, settled, last_report, last_telemetry;
    uint32_t imu_count, imu_updated, imu_errors;
    uint32_t duration_ms, distance_mm;
    uint32_t line_valid_since;
    uint8_t line_candidate;
    int32_t left_ticks, right_ticks;
    WHEEL_ODOMETRY odometry;
    const char *stop_reason;
    ER telemetry_error;
    bool started_once, force_status, imu_fault, line_candidate_valid;
    bool pwm_clock_started;
    uint32_t pwm_updated;
    float pwm_ramp_per_second;
    IMU_DATA imu;
    line_barcode_msg_t line;
    ER imu_er, line_er;
    float left, right;
    int corner_direction;
    int last_side;
    uint32_t side_seen;
    uint32_t corner_started, centre_since;
    bool corner_active, centre_pending;
    bool white_clear_pending, white_reported;
    uint32_t white_clear_since;
    unsigned steering_action;
    unsigned white_report_pattern, white_report_action;
    uint8_t pending_letter;          /* bit 7 = waiting, bits 4..0 = A=0..Z=25 */
    unsigned turn_stage, turn_command;
    uint32_t turn_started, turn_done;
    int32_t turn_left0, turn_right0;
    bool turn_done_valid;
    bool leaving_junction;
    /* S5 obstacle-avoidance hook (IF-10/IF-11, see s5_step below). */
    S5_EXEC s5;
    int32_t s5_left0, s5_right0;
} ALL_TEST;

/* S5 requests: HOLD replies once the robot has had this long to stop. */
#define S5_HOLD_SETTLE_MS  150u
/* ... and a move may take this long: 12.5-15 % PWM moves about
 * 8-10 cm/s, so an 80 cm leg 4 needs about 10 s (2026-11-05 run). */
#define S5_MOVE_TIMEOUT_MS 20000u
/* S5 pivots: one wheel drives, the other is stopped, so the moving wheel
 * runs on a circle of radius = the wheel spacing (centre to centre).  A
 * turn of D degrees is D x pi / 180 x S5_PIVOT_TRACK_CM of wheel path, at
 * that wheel's S5_*_TICKS_PER_CM.  This replaces S2's turn calibration x a
 * scale (measured with the old polled encoder): at 127 % it asked for 2618
 * ticks, about 104 deg, and the first turn overshot (2026-11-06).
 * [VERIFY: measure the spacing; turns long -> lower it, short -> raise] */
/* 12 (a guess) overshot; 10 cm (measured) turned about 10 deg short of
 * 90 (2026-11-07), so 10 x 90 / 80 = 11.25.  It is the effective pivot
 * radius, not the ruler distance: the stopped wheel also slips. */
#define S5_PIVOT_TRACK_CM 11.25f
/* ... at this PWM on the moving wheel.  12.5-15 % stalled mid-turn (the
 * stopped wheel drags sideways): step 3 stopped dead after 2137 ticks and
 * step 1 sat still for about 8 s (2026-11-06).  The 15 % cap only guarded
 * against the old polled encoder miscounting; interrupts count at any
 * speed.  [Tune] */
#define S5_PIVOT_PWM 20.0f
#define S5_PI 3.14159265f
/* Straight legs: the leading wheel's PWM is cut by this much per cm it is
 * ahead.  0.13 % per tick held the heading well before the encoder fix
 * (3 %/cm x 3.3 at 76 ticks/cm); at 117.8 ticks/cm that is 15 %/cm. */
#define S5_HEADING_KP 15.0f
/* Ticks per real cm for S5's moves, from the hand-push calibration
 * (S5_ENCODER_CAL) with the interrupt-driven encoder, 2026-11-06: three
 * 100 cm pushes gave left 117.9 / 119.4 / 116.2 and right 118.6 / 122.5 /
 * 119.9 (R/L 1.005-1.031).  These replace the old S5_DIST_SCALE (3.3) and
 * S5_RIGHT_RATIO guesses, which only made up for counts the 1 ms polled
 * encoder lost.  S2_*_TICKS_PER_CM (23.10 / 18.83) were also measured with
 * the polled encoder; Buddy 2 should re-measure them. */
#define S5_LEFT_TICKS_PER_CM  117.8f
#define S5_RIGHT_TICKS_PER_CM 120.3f
/* 1 = build the hand-push encoder calibration instead of the normal run
 * (see encoder_cal_task).  Keep 0 for normal all_test images. */
#ifndef S5_ENCODER_CAL
#define S5_ENCODER_CAL 0
#endif

static const char *const command_names[4] = {"LEFT", "RIGHT", "STRAIGHT", "U-TURN"};

static float approach_pwm(float current, float target, float step)
{
    if (target > current + step) return current + step;
    if (target < current - step) return current - step;
    return target;
}

static void apply_follow_pwm(ALL_TEST *t, float left, float right, uint32_t now)
{
    /* Physical wheel trim selected on hardware: straight = 12.5/14.2.
     * Preserve 10% on the inner right wheel when steering right. */
    if (right == ALL_BASE_PWM) right = ALL_RIGHT_BASE_PWM;
    uint32_t elapsed = t->pwm_clock_started ? now - t->pwm_updated : LINE_BARCODE_PERIOD_MS;
    /* A delayed cycle/resume must not turn into a sudden full-power step. */
    if (elapsed > 2u * LINE_BARCODE_PERIOD_MS) elapsed = 2u * LINE_BARCODE_PERIOD_MS;
    t->pwm_updated = now;
    t->pwm_clock_started = true;
    if (t->pwm_ramp_per_second > 0.0f) {
        float step = t->pwm_ramp_per_second * (float)elapsed / 1000.0f;
        t->left = approach_pwm(t->left, left, step);
        t->right = approach_pwm(t->right, right, step);
    } else {
        t->left = left;
        t->right = right;
    }
    drv_motor_set(t->left, t->right);
}

static void stop_test(ALL_TEST *t, const char *reason, unsigned state)
{
    if (ALL_TEST_CONTINUE_ON_SENSOR && state == S1_STATE_FAULT) {
        t->stop_reason = reason;
        return;
    }
    drv_motor_stop();
    t->left = t->right = 0.0f;
    s5_exec_set_following(&t->s5, FALSE);
    if (t->phase != STOPPED) {
        t->phase = STOPPED;
        t->terminal_state = state;
        t->stop_reason = reason;
        (void)line_service_enable(false);
        t->force_status = true;
        tm_printf((UB *)"[ALL TEST] STOPPED: %s; sample_dt=%lums line_fault=%u (1=line/GPIO 2=queue 4=timing); reboot to retry\n",
                  reason, (unsigned long)line_service_fault_interval_ms(),
                  (UINT)((t->line.data[2] >> LINE_BARCODE_FAULT_SHIFT)
                                  & LINE_BARCODE_ERR_MASK));
    }
}

static void wait_for_line(ALL_TEST *t)
{
    t->corner_active = t->centre_pending = false;
    t->last_side = 0;
    drv_motor_stop();
    t->left = t->right = 0.0f;
    s5_exec_set_following(&t->s5, FALSE);
    if (t->phase == LINE_WAIT) return;
    /* Keep sampling raw inputs while navigation is paused. This prevents
     * ordinary line absence from becoming S3's permanent search timeout.
     * Fault snapshots/events must be checked before calling this helper. */
    if (line_service_enable(false) < E_OK) {
        stop_test(t, "line recovery pause failed", S1_STATE_FAULT);
        return;
    }
    t->phase = LINE_WAIT;
    t->line_candidate_valid = false;
    t->stop_reason = "waiting for black line (auto-resume)";
    t->force_status = true;
    tm_printf((UB *)"[ALL TEST] line missing; motors off until a valid black line returns\n");
}

/* Remain RUNNING so IMU, queue, duration and manual stop checks still apply.
 * Active searches retain their direction through white gaps until real black
 * changes it. A new search without a recent side defaults left. */
static bool corner_step(ALL_TEST *t, uint32_t now)
{
    if (!ALL_TEST_CORNER_SEARCH || ALL_TEST_LINE_LAYOUT != LINE_BARCODE_LAYOUT_TRACK)
        return false;
    unsigned sensors = t->line.data[0] & 7u;
    unsigned state = t->line.data[1] & 7u;
    if (state == LINE_BARCODE_PAUSED || state == LINE_BARCODE_JUNCTION ||
        sensors == 5u || sensors == 7u) {
        stop_test(t, "ambiguous corner/junction; no route rule", S1_STATE_FAULT);
        return true;
    }
    if (sensors == 4u || sensors == 6u || sensors == 1u || sensors == 3u) {
        t->last_side = (sensors & 4u) ? -1 : 1;
        t->side_seen = now;
    }
    int gap_direction = t->corner_active ? t->corner_direction :
        (t->last_side != 0 &&
         (uint32_t)(now - t->side_seen) <= ALL_TEST_DIRECTION_HOLD_MS ? t->last_side : -1);
    int direction = sensors == 0u ? gap_direction : sensors == 4u ? -1 :
                    (sensors == 1u || sensors == 3u) ? 1 : 0;
    if (!t->corner_active) {
        if (direction == 0) return false;
        t->corner_active = true;
        t->centre_pending = false;
        t->corner_direction = direction;
        t->corner_started = now;
        t->stop_reason = "corner search";
        tm_printf((UB *)"[ALL TEST] corner search %s; timeout=%u ms (0=disabled)\n",
                  direction < 0 ? "left" : "right",
                  (UINT)ALL_TEST_CORNER_TIMEOUT_MS);
    }
    /* Changing direction never resets the total search deadline. */
#if ALL_TEST_CORNER_TIMEOUT_MS > 0
    if ((uint32_t)(now - t->corner_started) >= ALL_TEST_CORNER_TIMEOUT_MS) {
        stop_test(t, "corner search timed out", S1_STATE_FAULT);
        return true;
    }
#endif
    if (direction != 0 && direction != t->corner_direction) {
        t->corner_direction = direction;
        t->centre_pending = false;
        tm_printf((UB *)"[ALL TEST] corner search switched %s\n",
                  direction < 0 ? "left" : "right");
    }
    if (sensors == 6u && state != LINE_BARCODE_SEARCH) {
        t->corner_active = t->centre_pending = false;
        t->stop_reason = NULL;
        return false; /* Left+centre black: ordinary left steering. */
    }
    /* Keep moving at 90% straight PWM during centre confirmation. */
    if (sensors == 2u && state != LINE_BARCODE_SEARCH) {
        apply_follow_pwm(t, ALL_BASE_PWM * 0.9f, ALL_RIGHT_BASE_PWM * 0.9f, now);
        if (!t->centre_pending) {
            t->centre_pending = true;
            t->centre_since = now;
        }
        if ((uint32_t)(now - t->centre_since) >= ALL_TEST_CORNER_CENTRE_MS) {
            t->corner_active = t->centre_pending = false;
            t->stop_reason = NULL;
            tm_printf((UB *)"[ALL TEST] corner complete; centre line stable\n");
        }
        return true;
    }
    t->centre_pending = false;
    /* Immediately stop the inside wheel, ramp the outside wheel only. */
    if (t->corner_direction < 0) {
        t->left = 0.0f;
        apply_follow_pwm(t, 0.0f, ALL_BASE_PWM, now);
    } else {
        t->right = 0.0f;
        apply_follow_pwm(t, ALL_BASE_PWM, 0.0f, now);
    }
    return true;
}

/* S1 telemetry action for a decoded letter, from S3's current map. */
static unsigned barcode_action(unsigned letter)
{
    uint8_t command;
    if (line_service_lookup((uint8_t)letter, &command) != E_OK) return S1_BARCODE_NONE;
    return command == LINE_BARCODE_CMD_LEFT ? S1_BARCODE_LEFT :
           command == LINE_BARCODE_CMD_RIGHT ? S1_BARCODE_RIGHT :
           command == LINE_BARCODE_CMD_STRAIGHT ? S1_BARCODE_STRAIGHT : S1_BARCODE_UTURN;
}

/* Left encoder direction is unconfirmed on hardware; use travelled magnitude. */
static uint32_t tick_delta(int32_t now, int32_t start)
{
    int32_t delta = now - start;
    return (uint32_t)(delta < 0 ? -delta : delta);
}

static void begin_turn_stage(ALL_TEST *t, unsigned stage)
{
    t->turn_stage = stage;
    t->turn_left0 = t->left_ticks;
    t->turn_right0 = t->right_ticks;
}

/* At a C/R-both-black junction, consume the stored letter. True when a
 * mapped movement starts; false leaves the existing stop rule in charge. */
static bool start_junction_turn(ALL_TEST *t, uint32_t now)
{
    uint8_t command;
    if (!ALL_TEST_BARCODE_TURNS || (t->pending_letter & 0x80u) == 0u) return false;
    unsigned letter = t->pending_letter & LINE_BARCODE_LETTER_MASK;
    t->pending_letter = 0u;
    if (line_service_lookup((uint8_t)letter, &command) != E_OK) {
        tm_printf((UB *)"[ALL TEST] junction: letter '%c' has no mapping\n", 'A' + letter);
        return false;
    }
    t->turn_command = command & 3u;
    t->turn_started = now;
    begin_turn_stage(t, TURN_ADVANCE);
    t->stop_reason = "barcode junction movement";
    t->force_status = true;
    tm_printf((UB *)"[MOTION] junction reached: letter '%c' -> %s\n", 'A' + letter,
              command_names[t->turn_command]);
    return true;
}

static void finish_turn(ALL_TEST *t, uint32_t now)
{
    t->turn_stage = TURN_NONE;
    t->turn_done = now;
    t->turn_done_valid = true;
    t->leaving_junction = true;
    t->steering_action = 0u;
    t->white_clear_pending = false;
    t->white_reported = false;
    t->stop_reason = NULL;
    t->force_status = true;
    apply_follow_pwm(t, ALL_BASE_PWM, ALL_BASE_PWM, now);
    tm_printf((UB *)"[MOTION] turn complete; forward line following resumed\n");
}

/* Sensors are ignored until the movement ends; IMU, queue and pause checks
 * still run in all_test_step. */
static void turn_step(ALL_TEST *t, uint32_t now)
{
    if ((uint32_t)(now - t->turn_started) >= ALL_TEST_TURN_TIMEOUT_MS) {
        t->turn_stage = TURN_NONE;
        stop_test(t, "junction movement timed out", S1_STATE_FAULT);
        return;
    }
    if (t->turn_stage == TURN_ADVANCE) {
        float advance = ALL_TEST_TURN_ADVANCE_CM * S2_RIGHT_TICKS_PER_CM;
        if ((float)tick_delta(t->right_ticks, t->turn_right0) < advance) {
            apply_follow_pwm(t, ALL_BASE_PWM, ALL_BASE_PWM, now);
            return;
        }
        if (t->turn_command == LINE_BARCODE_CMD_STRAIGHT) {
            finish_turn(t, now);
            return;
        }
        begin_turn_stage(t, TURN_PIVOT);
    }
    /* Right pivot moves the left wheel; left pivot and U-turn the right. */
    bool right_turn = t->turn_command == LINE_BARCODE_CMD_RIGHT;
    float degrees = t->turn_command == LINE_BARCODE_CMD_UTURN ? 180.0f : 90.0f;
    float target = degrees * (right_turn ? S2_LEFT_TURN_TICKS_PER_DEGREE :
                                           S2_RIGHT_TURN_TICKS_PER_DEGREE);
    uint32_t moved = right_turn ? tick_delta(t->left_ticks, t->turn_left0) :
                                  tick_delta(t->right_ticks, t->turn_right0);
    if ((float)moved >= target) {
        finish_turn(t, now);
    } else if (right_turn) {
        t->right = 0.0f;
        apply_follow_pwm(t, ALL_RIGHT_TURN_LEFT_PWM, 0.0f, now);
    } else {
        t->left = 0.0f;
        apply_follow_pwm(t, 0.0f, ALL_BASE_PWM, now);
    }
}

static void white_follow_step(ALL_TEST *t, uint32_t now)
{
    unsigned sensors = t->line.data[0] & 3u; /* Ignore left entirely. */
    if ((t->line.data[1] & 7u) == LINE_BARCODE_PAUSED) {
        stop_test(t, "white tracking sensor service paused", S1_STATE_FAULT);
        return;
    }
    /* A completed turn may still leave the sensors on the same crossing.
     * Re-arm junction detection only after the settling period and exit. */
    if (t->leaving_junction && sensors != 3u
        && (uint32_t)(now - t->turn_done) >= ALL_TEST_TURN_SETTLE_MS)
        t->leaving_junction = false;

    t->corner_active = t->centre_pending = false;
    t->last_side = 0;
    if (sensors != 0u) {
        t->steering_action = sensors;
        t->white_clear_pending = false;
    } else if (t->steering_action != 0u) {
        if (!t->white_clear_pending) {
            t->white_clear_pending = true;
            t->white_clear_since = now;
        }
        if ((uint32_t)(now - t->white_clear_since) >= ALL_WHITE_CLEAR_MS) {
            t->steering_action = 0u;
            t->white_clear_pending = false;
        }
    }
    unsigned action = t->steering_action;
    if (!t->white_reported || sensors != t->white_report_pattern ||
        action != t->white_report_action) {
        tm_printf((UB *)"[WHITE STEER] C=%u R=%u action=%s%s\n",
                  (UINT)((sensors >> 1u) & 1u), (UINT)(sensors & 1u),
                  action == 0u ? "STRAIGHT" : action == 1u ? "RIGHT target=20/0" :
                  action == 2u ? "LEFT" : "STOP",
                  sensors == 0u && action != 0u ? " (confirming white 100ms)" : "");
        t->white_reported = true;
        t->white_report_pattern = sensors;
        t->white_report_action = action;
    }
    if (sensors == 3u && t->leaving_junction) {
        /* Still on the crossing just left by a movement. */
        apply_follow_pwm(t, ALL_BASE_PWM, ALL_BASE_PWM, now);
    } else if (sensors == 3u && start_junction_turn(t, now)) {
        turn_step(t, now);
    } else if (sensors == 3u) {
        stop_test(t, "centre and right both black", S1_STATE_FAULT);
    } else if (action == 1u) {
        t->right = 0.0f;
        t->stop_reason = "right black: turn right";
        apply_follow_pwm(t, ALL_RIGHT_TURN_LEFT_PWM, 0.0f, now);
    } else if (action == 2u) {
        t->left = 0.0f;
        t->stop_reason = "centre black: turn left";
        apply_follow_pwm(t, 0.0f, ALL_BASE_PWM, now);
    } else {
        t->stop_reason = NULL;
        apply_follow_pwm(t, ALL_BASE_PWM, ALL_BASE_PWM, now);
    }
}

/* ---- S5 obstacle-avoidance hook (added by Buddy 5; IF-10/IF-11) --------
   The shared executor (subsystem5/s5_executor.c) runs the hand-shake and
   calls back into the functions below, which drive with this loop's own
   PWM levels and encoder-counted pivots (as turn_step). */
typedef struct {
    ALL_TEST *t;
    uint32_t now;
} AT_CTX;

static void at_stop(void *p)
{
    ALL_TEST *t = ((AT_CTX *)p)->t;
    drv_motor_stop();
    t->left = t->right = 0.0f;
    t->pwm_clock_started = false; /* next move ramps up from rest */
}

/* Remember where the encoders were when the move began. */
static BOOL at_start_move(void *p, const S5_REQUEST *req)
{
    ALL_TEST *t = ((AT_CTX *)p)->t;
    (void)req;
    t->s5_left0 = t->left_ticks;
    t->s5_right0 = t->right_ticks;
    return true;
}

/* Distance a wheel has covered since the move began, in cm. */
static float at_left_cm(const ALL_TEST *t)
{
    return (float)tick_delta(t->left_ticks, t->s5_left0) / S5_LEFT_TICKS_PER_CM;
}

static float at_right_cm(const ALL_TEST *t)
{
    return (float)tick_delta(t->right_ticks, t->s5_right0) / S5_RIGHT_TICKS_PER_CM;
}

/* Moving-wheel ticks for an S5 pivot of 'deg' degrees. */
static float s5_pivot_ticks(UH deg, float ticks_per_cm)
{
    return (float)deg * S5_PI / 180.0f * S5_PIVOT_TRACK_CM * ticks_per_cm;
}

static UB at_run_move(void *p, const S5_REQUEST *req)
{
    AT_CTX *c = p;
    ALL_TEST *t = c->t;
    if (req->move == S5_MOVE_FWD) {
        /* Hold the heading: both wheels start at their straight PWM and the
           one that is ahead is slowed until the distances match. The
           open-loop 12.5/15 % pair drifted badly after every restart
           (2026-11-06: leg 2 left 72 cm, right 55 cm, about 80 deg off).
           Only ever slow the wheel that is ahead, never speed one up: the
           1 ms polled encoder miscounts the left wheel backwards above its
           straight PWM (2026-11-06: at 16-22.5 % it read backwards, the
           correction ran away and the robot veered right into the box). */
        float left_cm = at_left_cm(t), right_cm = at_right_cm(t);
        float err = S5_HEADING_KP * (right_cm - left_cm); /* > 0: right ahead */
        float left_pwm = ALL_BASE_PWM, right_pwm = ALL_RIGHT_BASE_PWM;
        if ((left_cm + right_cm) * 0.5f >= (float)req->value) return S5_EXEC_DONE;
        if (err > 0.0f) right_pwm -= err;
        else left_pwm += err;
        if (left_pwm < 0.0f) left_pwm = 0.0f;
        if (right_pwm < 0.0f) right_pwm = 0.0f;
        apply_follow_pwm(t, left_pwm, right_pwm, c->now);
    } else if (req->move == S5_MOVE_TURN_R) {
        /* Right pivot moves the left wheel (as turn_step). */
        if ((float)tick_delta(t->left_ticks, t->s5_left0) >=
            s5_pivot_ticks(req->value, S5_LEFT_TICKS_PER_CM)) return S5_EXEC_DONE;
        t->right = 0.0f;
        apply_follow_pwm(t, S5_PIVOT_PWM, 0.0f, c->now);
    } else {
        /* Left pivot moves the right wheel. */
        if ((float)tick_delta(t->right_ticks, t->s5_right0) >=
            s5_pivot_ticks(req->value, S5_RIGHT_TICKS_PER_CM)) return S5_EXEC_DONE;
        t->left = 0.0f;
        apply_follow_pwm(t, 0.0f, S5_PIVOT_PWM, c->now);
    }
    return S5_EXEC_RUNNING;
}

/* Motors are off straight away; S5_HOLD_SETTLE_MS lets the robot coast. */
static BOOL at_is_stopped(void *p)
{
    (void)p;
    return true;
}

/* Steering sensors (SIDE: C and R; L is the barcode scanner). */
#define AT_LINE_MASK (ALL_TEST_LINE_LAYOUT == LINE_BARCODE_LAYOUT_SIDE ? \
                      3u : LINE_BARCODE_SENSOR_MASK)

static UB at_line_sensors(void *p)
{
    ALL_TEST *t = ((AT_CTX *)p)->t;
    return t->line_er >= E_OK ? (UB)(t->line.data[0] & AT_LINE_MASK) : 0u;
}

static BOOL at_take_control(void *p)
{
    ALL_TEST *t = ((AT_CTX *)p)->t;
    t->turn_stage = TURN_NONE; /* obstacle wins over a barcode turn */
    t->corner_active = t->centre_pending = false;
    if (line_service_enable(false) < E_OK) {
        stop_test(t, "line pause for obstacle failed", S1_STATE_FAULT);
        return false;
    }
    t->stop_reason = "obstacle: S5 has control";
    t->force_status = true;
    tm_printf((UB *)"[ALL TEST] obstacle: S5 has control\n");
    return true;
}

/* Same clean-up as the end of a junction turn. */
static BOOL at_give_back(void *p)
{
    ALL_TEST *t = ((AT_CTX *)p)->t;
    if (line_service_enable(true) < E_OK) {
        stop_test(t, "line resume after obstacle failed", S1_STATE_FAULT);
        return false;
    }
    /* Rejoining at a slight angle can put both sensors on the line for a
     * moment; without this the follower read it as a junction and stopped
     * ("centre and right both black", 2026-11-06).  Same grace as after a
     * junction turn: drive straight until the sensors clear and
     * ALL_TEST_TURN_SETTLE_MS has passed. */
    t->leaving_junction = true;
    t->turn_done = ((AT_CTX *)p)->now;
    t->steering_action = 0u;
    t->white_clear_pending = false;
    t->white_reported = false;
    t->stop_reason = NULL;
    t->force_status = true;
    tm_printf((UB *)"[ALL TEST] obstacle handled; line following resumed\n");
    return true;
}

static void at_safe_stop(void *p, const char *reason)
{
    stop_test(((AT_CTX *)p)->t, reason, S1_STATE_FAULT);
}

static BOOL at_may_follow(void *p)
{
    return ((AT_CTX *)p)->t->turn_stage == TURN_NONE;
}

static const S5_EXEC_OPS at_s5_ops = {
    at_stop, at_start_move, at_run_move, at_is_stopped,
    at_line_sensors, at_take_control, at_give_back, at_safe_stop, at_may_follow,
    S5_HOLD_SETTLE_MS,
    S5_MOVE_TIMEOUT_MS, /* low power: an 80 cm leg takes ~10 s here */
    AT_LINE_MASK
};

/* Returns true while S5 has control, so line following is skipped. */
static bool s5_step(ALL_TEST *t, uint32_t now)
{
    AT_CTX c = {t, now};
    return s5_exec_step(&t->s5, &at_s5_ops, &c, now);
}

static void report_motor_outputs(void)
{
    MOTOR_DIAGNOSTICS d = {0};
    ER er = drv_motor_diagnostics(&d);
    tm_printf((UB *)"[MOTOR READBACK] check=%d GP8=%lu GP9=%lu GP10=%lu GP11=%lu CSR_L=%lu CSR_R=%lu\n",
              er, (unsigned long)(d.left_compare & 0xffffu),
              (unsigned long)(d.left_compare >> 16u),
              (unsigned long)(d.right_compare & 0xffffu),
              (unsigned long)(d.right_compare >> 16u),
              (unsigned long)d.left_csr, (unsigned long)d.right_csr);
}

static const char *imu_status_reason(const ALL_TEST *t, uint32_t now)
{
    if (t->imu_er != E_OK) return "IMU snapshot API failed";
    if (!t->imu.sensor_ok) return "IMU sensor reports not OK";
    if (!t->imu.calibrated) return "IMU calibration not ready";
    if (t->imu_count == 0u) return "IMU has no samples";
#if ALL_TEST_STOP_ON_COLLISION
    if (t->imu.motion_state == IMU_MOTION_COLLISION || t->imu.event == IMU_EVENT_COLLISION)
        return "IMU collision detected";
#endif
    if (t->imu.motion_state == IMU_MOTION_FAULT || t->imu.event == IMU_EVENT_FAULT)
        return "IMU processing fault";
    if ((uint32_t)(now - t->imu_updated) > ALL_IMU_STALE_MS)
        return "IMU samples stale (over 100ms)";
    return NULL;
}

static bool imu_ready(const ALL_TEST *t, uint32_t now)
{
    return imu_status_reason(t, now) == NULL;
}

static void report_telemetry(ALL_TEST *t, uint32_t now)
{
    if (!t->force_status && (uint32_t)(now - t->last_telemetry) < 1000u) return;
    S1_TELEMETRY packet;
    unsigned state = t->phase == STOPPED ? t->terminal_state :
                     t->phase == LINE_WAIT ? S1_STATE_RECOVERING :
                     t->phase == RUNNING && t->turn_stage != TURN_NONE ? S1_STATE_TURNING :
                     t->phase == RUNNING ? S1_STATE_FOLLOWING : S1_STATE_IDLE;
    unsigned sensors = t->line.data[0] & 7u;
    /* S3: L/C/R at bits 2/1/0; S1 wire protocol: L/C/R at bits 0/1/2. */
    unsigned wire_sensors = ((sensors & 4u) >> 2u) | (sensors & 2u) |
                            ((sensors & 1u) << 2u);
    s1_telemetry_init(&packet);
    (void)s1_telemetry_set(&packet, S1_FIELD_STATE, state);
    (void)s1_telemetry_set(&packet, S1_FIELD_FAULTS,
                         !t->imu_fault && imu_ready(t, now) ? 0 : S1_FAULT_IMU);
    (void)s1_telemetry_set(&packet, S1_FIELD_LINE_SENSORS, wire_sensors);
    (void)s1_telemetry_set(&packet, S1_FIELD_BARCODE, t->barcode);
    unsigned height = t->imu.ramp_height_mm;
    if (height > S1_HUMP_MAX) height = S1_HUMP_MAX;
    (void)s1_telemetry_set(&packet, S1_FIELD_HIGHEST_HUMP_MM, height);
    (void)s1_telemetry_set(&packet, S1_FIELD_DISTANCE_CM, t->distance_mm / 10u);
    /* Protocol v1 has no unavailable sentinel. Battery, speed, obstacle
     * and scan fields stay zero. Distance needs valid encoder scales;
     * height availability is printed separately on the serial console. */
    ER er = subsystem1_publish_telemetry(&packet);
    if (er != t->telemetry_error) {
        if (er == E_TMOUT)
            tm_printf((UB *)"[ALL TEST] telemetry queue full; waiting for MQTT to drain it\n");
        else if (er < E_OK)
            tm_printf((UB *)"[ALL TEST] telemetry queue=%d\n", er);
        else
            tm_printf((UB *)"[ALL TEST] telemetry queue accepting packets again\n");
        t->telemetry_error = er;
    }
    t->last_telemetry = now;
    t->force_status = false;
}

static void commands(ALL_TEST *t, uint32_t now)
{
    for (unsigned i = 0u; i < S1_QUEUE_DEPTH; ++i) {
        S1_COMMAND cmd;
        INT len = subsystem1_receive_command(&cmd, TMO_POL);
        if (len == E_TMOUT) break;
        if (len != S1_COMMAND_BYTES || !s1_command_valid(&cmd)) {
            stop_test(t, "MQTT command queue failed", S1_STATE_FAULT);
            break;
        }
        tm_printf((UB *)"[ALL TEST] command received=%u phase=%u\n",
                  (unsigned int)cmd.bytes[0], (unsigned int)t->phase);
        switch (cmd.bytes[0]) {
        case S1_COMMAND_STATUS:
            t->force_status = true;
            break;
        case S1_COMMAND_PAUSE:
            if (t->phase != STOPPED) {
                drv_motor_stop();
                t->left = t->right = 0.0f;
                t->corner_active = t->centre_pending = false;
                t->last_side = 0;
                t->white_clear_pending = false;
                t->steering_action = 0u;
                t->turn_stage = TURN_NONE;
                if (line_service_enable(false) < E_OK && !ALL_TEST_CONTINUE_ON_SENSOR) {
                    stop_test(t, "pause failed", S1_STATE_FAULT);
                } else {
                    t->phase = PAUSED;
                    t->stop_reason = "MQTT pause";
                    t->line_candidate_valid = false;
                    tm_printf((UB *)"[ALL TEST] MQTT pause\n");
                }
            }
            t->force_status = true;
            break;
        case S1_COMMAND_RESUME:
            if (t->phase == PAUSED && (ALL_TEST_CONTINUE_ON_SENSOR || imu_ready(t, now)) &&
                (uint32_t)(now - t->boot) >= ALL_START_MS) {
                if (line_service_enable(true) < E_OK && !ALL_TEST_CONTINUE_ON_SENSOR) {
                    stop_test(t, "resume failed", S1_STATE_FAULT);
                } else {
                    t->phase = SETTLING;
                    t->settled = now;
                    t->stop_reason = NULL;
                    t->line_candidate_valid = false;
                    tm_printf((UB *)"[ALL TEST] MQTT resume; checking line\n");
                }
            } else {
                tm_printf((UB *)"[ALL TEST] resume rejected: phase=%u or IMU not ready\n", t->phase);
            }
            t->force_status = true;
            break;
        case S1_COMMAND_RESET_STATS:
            /* S4 owns its statistics and has no reset API. */
            tm_printf((UB *)"[ALL TEST] reset statistics unsupported; reboot instead\n");
            t->force_status = true;
            break;
        }
    }
    /* This hardware profile uses the user's fixed A/F/Z/W route map.
     * Drain remote mapping messages without changing those assignments. */
    for (unsigned i = 0u; i < S1_QUEUE_DEPTH; ++i) {
        S1_MAPPING mapping;
        INT len = subsystem1_receive_mapping(&mapping, TMO_POL);
        if (len == E_TMOUT) break;
        if (len < E_OK) break;
        tm_printf((UB *)"[ALL TEST] MQTT map ignored: fixed A=LEFT F=RIGHT Z=STRAIGHT W=U-TURN\n");
    }
}

/* One step owns motor outputs, S3 events and the S1 command queue. The task
 * keeps servicing MQTT/status after a stop; RESUME cannot clear a fault. */
static void all_test_step(ALL_TEST *t, uint32_t now)
{
    uint8_t travelled_mm;
    drv_encoder_get_counts(&t->left_ticks, &t->right_ticks);
    if (!wheel_odometry_step(&t->odometry,
                            t->left_ticks,
                            t->right_ticks,
                            t->phase == RUNNING &&
                            !t->corner_active &&
                            t->turn_stage != TURN_PIVOT,
                            &travelled_mm)) {
        stop_test(t, "encoder distance out of range; check calibration", S1_STATE_FAULT);
    } else if (travelled_mm != 0u) {
        ER er = subsystem4_add_distance_mm(travelled_mm);
        if (er < E_OK) stop_test(t, "IMU distance input failed", S1_STATE_FAULT);
        /* Telemetry's centimetre field saturates instead of wrapping. */
        t->distance_mm += travelled_mm;
        if (t->distance_mm > S1_DISTANCE_MAX * 10u)
            t->distance_mm = S1_DISTANCE_MAX * 10u;
    }
    t->imu_er = subsystem4_get_latest(&t->imu);
    if (t->imu_er == E_OK && t->imu.sample_count != t->imu_count) {
        t->imu_count = t->imu.sample_count;
        t->imu_updated = now;
    }
    t->line_er = line_service_snapshot(&t->line);
    commands(t, now);
    if (t->duration_ms != 0u && t->started_once && t->phase != STOPPED &&
        (uint32_t)(now - t->started) >= t->duration_ms) {
        stop_test(t, "configured run time complete", S1_STATE_FINISHED);
    }
    if (ALL_TEST_CONTINUE_ON_SENSOR) {
        const char *reason = imu_status_reason(t, now);
        t->imu_fault = reason != NULL || t->imu.read_errors != t->imu_errors;
        if (reason != NULL) t->stop_reason = reason;
        t->imu_errors = t->imu.read_errors;
        if (t->phase == WAITING && (uint32_t)(now - t->boot) >= ALL_START_MS) {
            (void)line_service_enable(true);
            t->phase = RUNNING;
        }
        if (t->phase == SETTLING || t->phase == LINE_WAIT) t->phase = RUNNING;
        if (t->phase == RUNNING) {
            if (!t->started_once) {
                t->started = now;
                t->started_once = true;
            }
            for (unsigned i = 0; i < 24u; ++i) {
                line_barcode_msg_t event;
                ER er = line_service_receive(&event, TMO_POL);
                if (er == E_TMOUT) break;
                if (er < E_OK) { t->stop_reason = "line queue failed"; break; }
                unsigned code = (event.data[1] >> 3u) & 7u;
                if (code == LINE_BARCODE_EVT_INVALID) {
                    tm_printf((UB *)"[BARCODE REJECT] invalid frame: check start/stop, narrow/wide widths and white margins\n");
                }
                if (code == LINE_BARCODE_EVT_BARCODE) {
                    unsigned letter = event.data[2] & LINE_BARCODE_LETTER_MASK;
                    t->barcode = barcode_action(letter);
                    tm_printf((UB *)"[ALL TEST] barcode '%c' detected; continuing\n", 'A' + letter);
                } else if (code != 0u) {
                    tm_printf((UB *)"[ALL TEST] line event=%u; continuing\n", code);
                }
            }
            float left = ALL_BASE_PWM, right = ALL_BASE_PWM;
            bool line_valid = false;
            if (t->line_er == E_OK) {
                line_valid = line_follow_output(t->line.data[0], ALL_TEST_LINE_LAYOUT,
                                                ALL_BASE_PWM, ALL_INNER_PWM, &left, &right);
                if (!line_valid) {
                    t->stop_reason = "line absent/ambiguous; motors stopped";
                }
                /* Reset navigation faults/search timeouts so fresh steering can recover. */
                unsigned state = t->line.data[1] & 7u;
                if (state == LINE_BARCODE_FAULT || state == LINE_BARCODE_PAUSED ||
                    state == LINE_BARCODE_JUNCTION) (void)line_service_enable(true);
            } else {
                t->stop_reason = "line snapshot failed; motors stopped";
            }
            if (line_valid) {
                t->stop_reason = reason;
                apply_follow_pwm(t, left, right, now);
            } else {
                drv_motor_stop();
                t->left = t->right = 0.0f;
                t->pwm_clock_started = false;
            }
        }
        goto report_status;
    }
    if (t->phase == RUNNING || t->phase == SETTLING || t->phase == LINE_WAIT ||
        (t->phase == PAUSED && t->started_once)) {
        const char *reason = imu_status_reason(t, now);
        if (reason == NULL && t->imu.read_errors != t->imu_errors)
            reason = "IMU read-error count changed";
        if (reason != NULL) {
            t->imu_fault = true;
            stop_test(t, reason, S1_STATE_FAULT);
            /* Capture the triggering values once, after stopping motors.
             * Later live samples may recover to NORMAL; they must not
             * obscure the event/error that latched this stop. */
            tm_printf((UB *)"[ALL TEST] IMU stop data: api=%d age=%lu ms samples=%lu ok=%d calibrated=%d motion=%d event=%d errors=%lu baseline_errors=%lu\n",
                      t->imu_er, (unsigned long)(now - t->imu_updated),
                      (unsigned long)t->imu_count, t->imu.sensor_ok, t->imu.calibrated,
                      t->imu.motion_state, t->imu.event,
                      (unsigned long)t->imu.read_errors, (unsigned long)t->imu_errors);
            tm_printf((UB *)"[ALL TEST] IMU stop acceleration=(%d,%d,%d)mg baseline=(%d,%d,%d)mg\n",
                      t->imu.ax_mg, t->imu.ay_mg, t->imu.az_mg,
                      t->imu.baseline_ax_mg, t->imu.baseline_ay_mg, t->imu.baseline_az_mg);
        }
    }
    if (t->phase == WAITING) {
        if ((uint32_t)(now - t->boot) >= ALL_READY_MS) {
            t->imu_fault = true;
            stop_test(t, "IMU readiness timeout", S1_STATE_FAULT);
        } else if ((uint32_t)(now - t->boot) >= ALL_START_MS && imu_ready(t, now) &&
                   !s5_exec_startup_hold()) {
            t->imu_errors = t->imu.read_errors;
            if (line_service_enable(true) < E_OK) {
                stop_test(t, "line enable failed", S1_STATE_FAULT);
            } else {
                t->phase = SETTLING;
                t->settled = now;
            }
        }
    }
    if (t->phase == SETTLING && (uint32_t)(now - t->settled) >= 30u) {
        /* Enabling clears the old snapshot to PAUSED. Do not mistake a
         * fixed sleep for proof that the sensor task ran on the other core.
         * Motors stay off until a fresh sample from this session arrives. */
        if (t->line_er == E_TMOUT || (t->line_er == E_OK &&
            (t->line.data[1] & 7u) == LINE_BARCODE_PAUSED)) {
            if ((uint32_t)(now - t->settled) >= ALL_LINE_READY_MS)
                stop_test(t, "no fresh line sample after enable", S1_STATE_FAULT);
        } else if (t->line_er < E_OK) {
            stop_test(t, "line snapshot API failed at startup", S1_STATE_FAULT);
        } else {
            if (!t->started_once) {
                t->started = now;
                t->started_once = true;
                t->imu_errors = t->imu.read_errors;
            }
            t->phase = RUNNING;
        }
    }
    if (t->phase == RUNNING || t->phase == LINE_WAIT) {
        if (t->line_er < E_OK) {
            stop_test(t, "line snapshot stale/failed", S1_STATE_FAULT);
        } else if ((t->line.data[1] & 7u) == LINE_BARCODE_FAULT) {
            /* A newer fault must win over an older queued LOST event.
             * Never clear a sensor/queue/timing fault through auto-resume. */
            stop_test(t, "line sensor fault", S1_STATE_FAULT);
        } else if (t->phase == RUNNING) {
            bool lost_event = false;
            for (unsigned i = 0u; i < 24u; ++i) {
                line_barcode_msg_t event;
                ER er = line_service_receive(&event, TMO_POL);
                if (er == E_TMOUT) break;
                if (er < E_OK) {
                    stop_test(t, "line queue failed", S1_STATE_FAULT); break;
                }
                unsigned code = (event.data[1] >> 3u) & 7u;
                if (code == LINE_BARCODE_EVT_INVALID) {
                    tm_printf((UB *)"[BARCODE REJECT] invalid frame: check start/stop, narrow/wide widths and white margins\n");
                }
                if (code == LINE_BARCODE_EVT_LOST) {
                    lost_event = true;
                    if (!ALL_TEST_CORNER_SEARCH && !ALL_TEST_WHITE_FOLLOW) {
                        drv_motor_stop();
                        t->left = t->right = 0.0f;
                    }
                    continue; /* Drain newer faults before resetting S3. */
                }
                if (code == LINE_BARCODE_EVT_FAULT ||
                    (!ALL_TEST_WHITE_FOLLOW && (code == LINE_BARCODE_EVT_JUNCTION ||
                                               code == LINE_BARCODE_EVT_COMMAND))) {
                    stop_test(t, "line fault or junction", S1_STATE_FAULT); break;
                }
                if (code == LINE_BARCODE_EVT_BARCODE) {
                    unsigned letter = event.data[2] & LINE_BARCODE_LETTER_MASK;
                    unsigned action = barcode_action(letter);
                    t->barcode = action;
                    /* A newer barcode replaces any letter still waiting. */
                    t->pending_letter = (uint8_t)(0x80u | letter);
                    tm_printf((UB *)"[BARCODE -> MOTION] decoded=%c queued=%s; execute at next C/R junction\n",
                              'A' + letter, action == S1_BARCODE_NONE ? "no mapping" :
                              command_names[action - S1_BARCODE_LEFT]);
                }
            }
            unsigned state = t->line.data[1] & 7u;
            if (t->phase == RUNNING) {
                if (s5_step(t, now)) {
                    /* S5 obstacle avoidance owns the motors this step. */
                } else if (t->turn_stage != TURN_NONE) {
                    turn_step(t, now);
                } else if (ALL_TEST_WHITE_FOLLOW) {
                    white_follow_step(t, now);
                } else if (corner_step(t, now)) {
                    /* Corner helper owns PWM; common reporting still runs. */
                } else if (state == LINE_BARCODE_PAUSED || state == LINE_BARCODE_JUNCTION) {
                    stop_test(t, "line paused unexpectedly or junction", S1_STATE_FAULT);
                } else if (lost_event || state == LINE_BARCODE_SEARCH) {
                    wait_for_line(t);
                } else {
                    float left, right;
                    if (line_follow_output(t->line.data[0], ALL_TEST_LINE_LAYOUT,
                                            ALL_BASE_PWM, ALL_INNER_PWM, &left, &right)) {
                        apply_follow_pwm(t, left, right, now);
                    } else {
                        unsigned sensors = t->line.data[0] & LINE_BARCODE_SENSOR_MASK;
                        bool absent = ALL_TEST_LINE_LAYOUT == LINE_BARCODE_LAYOUT_SIDE ?
                                      (sensors & 3u) == 0u : sensors == 0u;
                        if (absent) wait_for_line(t);
                        else stop_test(t, "ambiguous L/C/R pattern or junction", S1_STATE_FAULT);
                    }
                }
            }
        }
    }
    if (t->phase == LINE_WAIT) {
        float left, right;
        uint8_t sensors = t->line.data[0] & LINE_BARCODE_SENSOR_MASK;
        bool valid = line_follow_output(sensors, ALL_TEST_LINE_LAYOUT,
                                        ALL_BASE_PWM, ALL_INNER_PWM, &left, &right);
        if (!valid) {
            t->line_candidate_valid = false;
        } else if (!t->line_candidate_valid || t->line_candidate != sensors) {
            t->line_candidate = sensors;
            t->line_candidate_valid = true;
            t->line_valid_since = now;
        } else if ((uint32_t)(now - t->line_valid_since) >= ALL_LINE_REACQUIRE_MS) {
            if (line_service_enable(true) < E_OK) {
                stop_test(t, "line recovery enable failed", S1_STATE_FAULT);
            } else {
                t->phase = SETTLING;
                t->settled = now;
                t->line_candidate_valid = false;
                t->stop_reason = NULL;
                t->force_status = true;
                tm_printf((UB *)"[ALL TEST] black line stable; automatically resuming\n");
            }
        }
    }
report_status:
    if ((uint32_t)(now - t->last_report) >= 500u) {
        unsigned sensors = t->line.data[0] & 7u;
        for (unsigned run = 0u; run < 64u; ++run) {
            uint32_t duration = 0u, dropped = 0u;
            unsigned black = 0u;
            if (!line_service_barcode_run(&duration, &black, &dropped)) break;
            tm_printf((UB *)"[BARCODE RUN] %s duration=%lums under_40ms=%u dropped=%lu (white may be margin)\n",
                      black ? "BLACK" : "WHITE", (unsigned long)duration,
                      (UINT)(duration < 40u), (unsigned long)dropped);
        }
        if (ALL_TEST_LINE_LAYOUT == LINE_BARCODE_LAYOUT_SIDE) {
            if (t->phase != RUNNING)
                tm_printf((UB *)"[BARCODE INFO] phase=%u; last recorded counts/result below; scanning may be inactive\n",
                          t->phase);
            unsigned raw_level = 0u, rises = 0u, falls = 0u;
            line_service_barcode_raw(&raw_level, &rises, &falls);
            tm_printf((UB *)"[BARCODE RAW] GP28=%u rises=%u falls=%u (before debounce, session totals)\n",
                      raw_level, rises, falls);
            unsigned current_black = 0u;
            uint32_t current_age = 0u;
            line_service_barcode_level(&current_black, &current_age);
            tm_printf((UB *)"[BARCODE LEVEL] GP28=%s held=%lums (ongoing run, not completed)\n",
                      current_black ? "BLACK" : "WHITE", (unsigned long)current_age);
            unsigned black_bars = 0u, white_runs = 0u;
            uint8_t decoded_letter = 0xffu;
            line_service_barcode_debug(&black_bars, &white_runs, &decoded_letter);
            tm_printf((UB *)"[BARCODE COUNT] black_lines=%u internal_white_spaces=%u decoded=%c (session totals; gaps >=1s excluded)\n",
                      black_bars, white_runs,
                      decoded_letter < 26u ? 'A' + decoded_letter : '-');
            unsigned scan_state = t->line.data[1] & 7u;
            tm_printf((UB *)"[BARCODE STATUS] GP28 debounced=%s scanning=%u service_state=%u pending=%c (500ms snapshot)\n",
                      (sensors & LINE_BARCODE_LEFT) ? "BLACK" : "WHITE",
                      (UINT)(scan_state == LINE_BARCODE_SCANNING), scan_state,
                      (t->pending_letter & 0x80u) ?
                          'A' + (t->pending_letter & LINE_BARCODE_LETTER_MASK) : '-');
        }
        tm_printf((UB *)"[ALL TEST] phase=%u L=%u C=%u R=%u line_er=%d IMU ok=%d calibrated=%d state=%d samples=%lu PWM=%d.%d/%d.%d ticks=%ld/%ld height=%umm valid=%d status=%s\n",
                  t->phase, (UINT)((sensors >> 2u) & 1u), (UINT)((sensors >> 1u) & 1u),
                  (UINT)(sensors & 1u), t->line_er, t->imu.sensor_ok,
                  t->imu.calibrated, t->imu.motion_state, (unsigned long)t->imu_count,
                  (int)t->left, ((int)(t->left * 10.0f)) % 10,
                  (int)t->right, ((int)(t->right * 10.0f)) % 10,
                  (long)t->left_ticks, (long)t->right_ticks,
                  (UINT)t->imu.ramp_height_mm, t->imu.ramp_height_valid,
                  t->stop_reason ? t->stop_reason : "none");
        t->last_report = now;
        if (ALL_TEST_WHITE_FOLLOW) report_motor_outputs();
    }
    report_telemetry(t, now);
}

#if S5_ENCODER_CAL
/* Hand-push encoder calibration (S5_ENCODER_CAL 1): the motors stay off
 * and both wheels' tick counts since reset are printed twice a second.
 * Push the robot slowly and straight along a 100 cm tape line; the final
 * counts / 100 are each wheel's ticks per real cm. */
#define S5_CAL_PUSH_CM   100
#define S5_CAL_REPORT_MS 500u

static void encoder_cal_task(void)
{
    int32_t left0, right0, left, right, dl, dr, ratio_x1000;
    SYSTIM now;
    uint32_t last = 0u;

    drv_motor_stop();
    drv_encoder_get_counts(&left0, &right0);
    tm_printf((UB *)"[ENC CAL] encoder calibration: motors OFF\n"
                    "[ENC CAL] push the robot slowly and straight along a %d cm line,\n"
                    "[ENC CAL] then read the last line: ticks/cm = count / %d\n",
              S5_CAL_PUSH_CM, S5_CAL_PUSH_CM);
    for (;;) {
        drv_encoder_poll();
        if (tk_get_tim(&now) == E_OK &&
            (uint32_t)(now.lo - last) >= S5_CAL_REPORT_MS) {
            drv_encoder_get_counts(&left, &right);
            dl = left - left0;
            dr = right - right0;
            ratio_x1000 = (dl != 0) ? (dr * 1000) / dl : 0;
            if (ratio_x1000 < 0) ratio_x1000 = -ratio_x1000;
            /* Integers only: ticks/cm = count / 100 printed as x.yy */
            tm_printf((UB *)"[ENC CAL] left=%ld right=%ld | if %d cm: L=%ld.%02ld R=%ld.%02ld ticks/cm, R/L=%ld.%03ld\n",
                      (long)dl, (long)dr, S5_CAL_PUSH_CM,
                      (long)(dl / S5_CAL_PUSH_CM), (long)((dl < 0 ? -dl : dl) % S5_CAL_PUSH_CM),
                      (long)(dr / S5_CAL_PUSH_CM), (long)((dr < 0 ? -dr : dr) % S5_CAL_PUSH_CM),
                      (long)(ratio_x1000 / 1000), (long)(ratio_x1000 % 1000));
            last = now.lo;
        }
        (void)tk_dly_tsk(S2_ENCODER_POLL_MS);
    }
}
#endif

static void all_task(INT stacd, void *exinf)
{
    ALL_TEST test = {0};
    SYSTIM now;
    uint32_t last_control;
    (void)stacd;
    (void)exinf;
#if S5_ENCODER_CAL
    encoder_cal_task(); /* never returns */
#endif
    tm_printf((UB *)"[ALL TEST] S1 MQTT + S2 motors + S3 line/barcode + S4 IMU + S5 obstacle scan\n[BARCODE BUILD] GP28 digital; cumulative counts; raw edges enabled\n[BARCODE DEBUG] duration logging enabled; minimum element=40ms; target=A\n"
              "[ALL TEST] keep stationary for IMU calibration; starts after 5s\n"
              "[ALL TEST] straight PWM left=12.5%% right=15%%; inner=6.25%%; GP28=left GP27=centre GP0=right; 1=black\n"
              "[ALL TEST] telemetry: unmeasured battery/speed/obstacle/scan fields=0 placeholders\n");
    if (ALL_TEST_WHITE_FOLLOW)
        tm_printf((UB *)"[ALL TEST] WHITE TRACK: GP27 centre + GP0 right; GP28 ignored for steering\n"
                  "[ALL TEST] C/R=0/0 straight, 0/1 right (left motor 20%%), 1/0 left, 1/1 STOP; 1=black\n");
    else
        tm_printf((UB *)"[ALL TEST] sensor response=%s; missing/invalid line stops motors; valid black auto-resumes; manual pause remains active\n",
              ALL_TEST_CONTINUE_ON_SENSOR ? "LOG ONLY (keep motors moving)" : "STOP/RECOVER");
    tm_printf((UB *)"[ALL TEST] layout=%s\n",
              ALL_TEST_LINE_LAYOUT == LINE_BARCODE_LAYOUT_TRACK ?
              "TRACK (three line sensors; barcode decoding disabled)" : "SIDE (offset scanner)");
    tm_printf((UB *)"[ALL TEST] barcode map: A=LEFT F=RIGHT Z=STRAIGHT W=U-TURN; other letters recognised only\n");
    if (tk_get_tim(&now) < E_OK) {
        drv_motor_stop();
        tk_slp_tsk(TMO_FEVR);
        return;
    }
    test.boot = now.lo;
    last_control = now.lo - LINE_BARCODE_PERIOD_MS;
    test.duration_ms = ALL_TEST_DURATION_MS;
    test.pwm_ramp_per_second = ALL_TEST_PWM_RAMP_PER_SECOND;
    tm_printf((UB *)"[ALL TEST] smooth motor start: 100 PWM percentage points/second\n");
    drv_encoder_get_counts(&test.left_ticks, &test.right_ticks);
    wheel_odometry_init(&test.odometry, S2_LEFT_TICKS_PER_CM,
                        S2_RIGHT_TICKS_PER_CM, test.left_ticks, test.right_ticks);
    if (!test.odometry.available)
        tm_printf((UB *)"[ALL TEST] height/distance unavailable: set measured S2_LEFT/RIGHT_TICKS_PER_CM; raw ticks are logged\n");
    tm_printf((UB *)"[ALL TEST] run limit=%lu ms (0=continuous)\n",
              (unsigned long)test.duration_ms);
    test.force_status = true;
    for (;;) {
        drv_encoder_poll();
        if (tk_get_tim(&now) < E_OK) {
            drv_motor_stop();
            stop_test(&test, "clock failed", S1_STATE_FAULT);
            tk_slp_tsk(TMO_FEVR);
            return;
        }
        if ((uint32_t)(now.lo - last_control) >= LINE_BARCODE_PERIOD_MS) {
            all_test_step(&test, now.lo);
            last_control = now.lo;
        }
        if (tk_dly_tsk(S2_ENCODER_POLL_MS) < E_OK) {
            drv_motor_stop();
            stop_test(&test, "control wait failed", S1_STATE_FAULT);
            tk_slp_tsk(TMO_FEVR);
            return;
        }
    }
}

ER all_buddies_test_start(void)
{
    ER er = drv_motor_init();
    MOTOR_DIAGNOSTICS diagnostics;
    if (er == E_OK) er = drv_motor_diagnostics(&diagnostics);
    if (er == E_OK) er = drv_encoder_init();
    /* Create all objects before any application task is started. */
    if (er == E_OK) er = subsystem1_init();
    if (er == E_OK) er = line_service_init();
    if (er == E_OK) er = line_service_calibrate(7u, 0u);
    if (er == E_OK) er = line_service_set_layout(ALL_TEST_LINE_LAYOUT);
    /* Only these four letters request junction movements. */
    if (er == E_OK) er = line_service_set_mapping('A', 'F', 'Z', 'W');
    if (er == E_OK) er = subsystem4_init();
    if (er == E_OK) er = subsystem5_init();
    if (er == E_OK) er = subsystem1_start();
    if (er == E_OK) er = line_service_start();
    if (er == E_OK) er = subsystem4_start();
    if (er == E_OK) er = subsystem5_start();
    if (er == E_OK) {
        ID tid = app_start_task(all_task, 15, TP_PRC2, "all_buddies_test");
        er = tid < E_OK ? tid : E_OK;
    }
    if (er < E_OK) drv_motor_stop();
    return er;
}
#endif
