/* Adapter around the existing S3/S4 public interfaces. */
#include "subsystem2.h"
#include "robot_control.h"

#if APP_ROBOT_INTEGRATED && !APP_TEST
#include "../subsystem3/line_service.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem5/s5_executor.h"
#include "../subsystem1/subsystem1.h"

static bool g_b_armed, g_b_halted, g_b_line_halt, g_b_turning, g_b_configured;
/* #8 straddle line model (S2_LINE_STRADDLE); runtime so host tests can
   exercise both models. */
static bool     g_b_straddle = S2_LINE_STRADDLE;
static unsigned g_steer_action; /* 0 straight, 1 right, 2 left */
static unsigned g_steer_candidate;
static uint32_t g_candidate_since, g_action_since, g_clear_since,
    g_junction_done;
static bool    g_b_clear_pending, g_b_leaving_junction;
static uint8_t g_pending_letter; /* bit 7 = waiting, bits 4..0 = A..Z */
/* #23 lost-line watch (straddle model, see S2_LINE_LOST_MM). */
static bool g_b_lost_watch, g_b_lost_probe, g_b_lost_stop, g_b_lost_seen_black;
static unsigned g_lost_side;          /* g_steer_action that was clearing */
static int32_t  g_lost_l0, g_lost_r0; /* wheel ticks when the watch began */
static uint32_t g_lost_last_step;     /* last straddle_step, for gaps */
static uint32_t g_lost_max_mm;        /* longest cleared watch this second */
static void     straddle_reset_steer(void);
#define TURN_ADVANCE 1u
#define TURN_PIVOT   2u
static unsigned g_turn_stage, g_turn_cmd;
static int32_t  g_uturn_r0; /* right ticks when the U-turn began */
static int32_t  g_pivot_l0,
    g_pivot_r0; /* wheel ticks when a left/right pivot began */
/* #48 Ready state: arm only once START has arrived (or g_b_auto_start is set).
   A START that arrives while still initialising is kept until Ready. */
static bool g_b_auto_start = S2_AUTO_START, g_b_start_requested,
            g_b_ready_reported;

static uint32_t g_last_imu_sample, g_imu_count, g_imu_errors, g_resumed_at,
    g_turn_started;
static uint32_t g_rearm_since;
static int32_t  g_rearm_l, g_rearm_r;
/* S5 obstacle-avoidance hook state (see avoid_step below). */
static S5_EXEC g_vc_s5;

/* Line stops (lost, line fault, bad snapshot) may re-arm by
   themselves; any other stop is a safety stop and latches until reboot. */
static void stop_robot(MOTION_CONTROLLER * p_ctl, uint32_t now,
                       int32_t left_ticks, int32_t right_ticks,
                       const char * p_reason, bool b_recoverable)
{
    MOTION_COMMAND stop = {MOTION_CMD_STOP, 0.0f};
    (void)motion_control_command(p_ctl, &stop, now, left_ticks, right_ticks);
    if (!g_b_halted)
    {
        /* Tell S5 the robot is no longer following (no new obstacles). */
        s5_exec_set_following(&g_vc_s5, FALSE);
        (void)line_service_enable(false);
        LOG_WARN(TAG_S2_ROBOT, "stopped: %s; %s", p_reason,
                 b_recoverable ? "re-arms when back on the line"
                               : "reboot to re-arm");
        g_b_line_halt = b_recoverable;
        g_rearm_since = now;
        g_rearm_l     = left_ticks;
        g_rearm_r     = right_ticks;
    }
    else if (!b_recoverable)
    {
        g_b_line_halt = false;
    }
    else
    {
        /* No action needed in any other case. */
    }
    g_b_halted = true;
}

static void halt(MOTION_CONTROLLER * p_ctl, uint32_t now, int32_t left_ticks,
                 int32_t right_ticks, const char * p_reason)
{
    stop_robot(p_ctl, now, left_ticks, right_ticks, p_reason, false);
}

static void halt_line(MOTION_CONTROLLER * p_ctl, uint32_t now,
                      int32_t left_ticks, int32_t right_ticks,
                      const char * p_reason)
{
    stop_robot(p_ctl, now, left_ticks, right_ticks, p_reason, true);
}

/* Paused S3 snapshots still carry calibrated sensor bits. Re-arm only after
   centre and right have both seen black, with the wheels still, for
   S2_LINE_REARM_MS; moving the robot by hand restarts the wait. */
static bool line_rearm_ready(uint32_t now, int32_t left_ticks,
                             int32_t right_ticks)
{
    line_barcode_msg_t msg  = {{0u, 0u, 0u}};
    ER                 ercd = line_service_snapshot(&msg);
    /* Straddle (#8): both-white is the normal on-line reading, so any healthy
       snapshot will do; the S3 model needs centre + right on black. */
    bool b_on_line = g_b_straddle ? (msg.data[1] & 7u) != LINE_BARCODE_FAULT
                                  : (msg.data[0] & 3u) == 3u;
    /* After a lost-line stop both white proves nothing: wait until C or R
       has seen the tape (the robot put back across it). */
    if (g_b_lost_stop && ercd >= E_OK && (msg.data[0] & 3u) != 0u)
    {
        g_b_lost_seen_black = true;
    }
    bool b_ready = ercd >= E_OK && b_on_line &&
                   (!g_b_lost_stop || g_b_lost_seen_black) &&
                   left_ticks == g_rearm_l && right_ticks == g_rearm_r;

    if (!b_ready)
    {
        g_rearm_since = now;
        g_rearm_l     = left_ticks;
        g_rearm_r     = right_ticks;
        return false;
    }
    return (uint32_t)(now - g_rearm_since) >= S2_LINE_REARM_MS;
}

static bool     g_b_line_unsafe_active;
static uint32_t g_line_unsafe_since;

/* A FAULT stops at once. Stale, paused, searching or absent-position
   snapshots must persist for S2_LINE_UNSAFE_MS before stopping, so one late
   or debouncing sample cannot latch a stop. S3 queues LOST after 100 ms. */
static bool line_snapshot_ok(MOTION_CONTROLLER * p_ctl, uint32_t now,
                             int32_t left_ticks, int32_t right_ticks, ER ercd,
                             const line_barcode_msg_t * p_msg,
                             bool                       b_need_position)
{
    unsigned state   = p_msg->data[1] & 7u;
    unsigned pos     = (p_msg->data[0] >> 3u) & 7u;
    bool     b_fault = ercd >= E_OK && state == LINE_BARCODE_FAULT;

    if (ercd >= E_OK && !b_fault && state != LINE_BARCODE_SEARCH &&
        state != LINE_BARCODE_PAUSED && (!b_need_position || pos < 6u))
    {
        g_b_line_unsafe_active = false;
        return true;
    }
    if (!g_b_line_unsafe_active)
    {
        g_b_line_unsafe_active = true;
        g_line_unsafe_since    = now;
    }
    if (b_fault || (uint32_t)(now - g_line_unsafe_since) >= S2_LINE_UNSAFE_MS)
    {
        LOG_WARN(TAG_S2_ROBOT, "line er=%d state=%u pos=%u sensors=%u",
                 (int)ercd, state, pos, (unsigned)(p_msg->data[0] & 7u));
        halt_line(p_ctl, now, left_ticks, right_ticks,
                  "line snapshot unsafe or stale");
    }
    return false;
}

/* ---- S5 obstacle-avoidance hook (added by Buddy 5; IF-10/IF-11) --------
   The VC stays the only motor commander. S5 only asks, through its request
   word; the shared executor (subsystem5/s5_executor.c) runs the hand-shake
   and calls back into the functions below, which drive through the motion
   controller. */
typedef struct
{
    MOTION_CONTROLLER * ctl;
    uint32_t            now;
    int32_t             l, r;
} VC_CTX;

static void vc_stop(void * p_arg)
{
    VC_CTX *       p_ctx = p_arg;
    MOTION_COMMAND stop  = {MOTION_CMD_STOP, 0.0f};
    (void)motion_control_command(p_ctx->ctl, &stop, p_ctx->now, p_ctx->l,
                                 p_ctx->r);
}

static BOOL vc_start_move(void * p_arg, const S5_REQUEST * p_req)
{
    VC_CTX *       p_ctx = p_arg;
    MOTION_COMMAND cmd;
    motion_control_step(p_ctx->ctl, p_ctx->now, p_ctx->l,
                        p_ctx->r); /* STOPPING -> IDLE */
    cmd.type  = p_req->move == S5_MOVE_FWD      ? MOTION_CMD_FORWARD_CM
                : p_req->move == S5_MOVE_TURN_L ? MOTION_CMD_TURN_LEFT_DEG
                                                : MOTION_CMD_TURN_RIGHT_DEG;
    cmd.value = (float)p_req->value;
    /* all_test's S5 moves: heading-held 12.5/15 % legs, 20 % pivots; a
       turn with the spin bit turns on the spot instead, and a leg with the
       reverse bit drives backwards (23 Nov). */
    motion_control_set_profile(
        p_ctx->ctl, (p_req->b_spin && p_req->move != S5_MOVE_FWD)
                        ? MOTION_PROFILE_SPIN
                    : (p_req->b_reverse && p_req->move == S5_MOVE_FWD)
                        ? MOTION_PROFILE_REVERSE
                        : MOTION_PROFILE_AVOID);

    return motion_control_command(p_ctx->ctl, &cmd, p_ctx->now, p_ctx->l,
                                  p_ctx->r) == MOTION_RESULT_OK;
}

static UB vc_run_move(void * p_arg, const S5_REQUEST * p_req)
{
    VC_CTX * p_ctx = p_arg;
    (void)p_req;
    /* The motion controller runs forward moves and pivots itself. */
    return (p_ctx->ctl->status.state == MOTION_STATE_IDLE &&
            p_ctx->ctl->status.motion_complete)
               ? S5_EXEC_DONE
               : S5_EXEC_RUNNING;
}

static BOOL vc_is_stopped(void * p_arg)
{
    VC_CTX * p_ctx = p_arg;
    return p_ctx->ctl->status.state == MOTION_STATE_IDLE;
}

/* Steering sensors (SIDE layout: C and R; L is the barcode scanner). */
#define VC_LINE_MASK \
    (S2_LINE_LAYOUT == LINE_BARCODE_LAYOUT_SIDE ? 3u : LINE_BARCODE_SENSOR_MASK)

static UB vc_line_sensors(void * p_arg)
{
    line_barcode_msg_t msg = {{0u, 0u, 0u}};
    (void)p_arg;
    return line_service_snapshot(&msg) >= E_OK
               ? (UB)(msg.data[0] & VC_LINE_MASK)
               : 0u;
}

static BOOL vc_take_control(void * p_arg)
{
    VC_CTX * p_ctx = p_arg;
    g_b_turning    = false; /* obstacle wins over a barcode turn */
    g_turn_stage   = 0u;
    if (line_service_enable(false) < E_OK)
    {
        halt(p_ctx->ctl, p_ctx->now, p_ctx->l, p_ctx->r, "line pause failed");
        return false;
    }
    LOG_INFO(TAG_S2_ROBOT, "obstacle: S5 has control");
    return true;
}

/* Same steps as the end of a barcode turn. */
static BOOL vc_give_back(void * p_arg)
{
    VC_CTX * p_ctx = p_arg;
    if (line_service_enable(true) < E_OK)
    {
        halt(p_ctx->ctl, p_ctx->now, p_ctx->l, p_ctx->r, "line resume failed");
        return false;
    }
    g_resumed_at           = p_ctx->now;
    g_b_line_unsafe_active = false;
    /* Rejoining at an angle can put C and R both on the line for a moment;
       as all_test, treat it like the crossing just left by a turn. */
    g_b_leaving_junction = g_b_straddle;
    g_junction_done      = p_ctx->now;
    g_steer_action = g_steer_candidate = 0u;
    g_b_clear_pending                  = false;
    LOG_INFO(TAG_S2_ROBOT, "obstacle handled, line following");
    return true;
}

static void vc_safe_stop(void * p_arg, const char * p_reason)
{
    VC_CTX * p_ctx = p_arg;
    halt(p_ctx->ctl, p_ctx->now, p_ctx->l, p_ctx->r, p_reason);
}

static BOOL vc_may_follow(void * p_arg)
{
    (void)p_arg;
    return !g_b_turning;
}

static const S5_EXEC_OPS vc_s5_ops = {
    vc_stop,
    vc_start_move,
    vc_run_move,
    vc_is_stopped,
    vc_line_sensors,
    vc_take_control,
    vc_give_back,
    vc_safe_stop,
    vc_may_follow,
    S2_AVOID_HOLD_SETTLE_MS,  /* all_test: let the robot coast to a stop */
    S2_AVOID_MOVE_TIMEOUT_MS, /* all_test: 20 s per leg or pivot */
    VC_LINE_MASK};

/* Returns true while S5 has control, so line following is skipped. */
static bool avoid_step(MOTION_CONTROLLER * p_ctl, uint32_t now,
                       int32_t left_ticks, int32_t right_ticks)
{
    VC_CTX ctx = {p_ctl, now, left_ticks, right_ticks};
    return s5_exec_step(&g_vc_s5, &vc_s5_ops, &ctx, now);
}

/* #72 MQTT link. PAUSE stops at once and blocks line re-arm until RESUME;
   RESUME restarts only on a healthy line and IMU; safety stops still need a
   reboot. The letter map arrives over MQTT; until then junctions go straight. */
static bool          g_b_paused, g_b_map_received, g_b_status_requested;
static unsigned char g_map_letters[S1_MAP_SLOTS]; /* 0..25 = A..Z */
static unsigned      g_last_letter;               /* 0 none, 1..26 = A..Z */
static IMU_DATA      g_last_imu;
static bool          g_b_have_imu, g_b_in_avoid;
static uint32_t      g_last_telemetry;
static ER            g_telemetry_error;
static int32_t       g_odo_l, g_odo_r;
static bool          g_b_odo_started;
static float         g_distance_cm;

static void vc_stop_motors(MOTION_CONTROLLER * p_ctl, uint32_t now,
                           int32_t left_ticks, int32_t right_ticks)
{
    MOTION_COMMAND stop = {MOTION_CMD_STOP, 0.0f};
    (void)motion_control_command(p_ctl, &stop, now, left_ticks, right_ticks);
}

/* The same healthy-IMU test that keeps the robot running. */
static bool imu_healthy(uint32_t now)
{
    return g_b_have_imu && g_last_imu.sensor_ok && g_last_imu.calibrated &&
           (!g_b_armed || g_last_imu.read_errors == g_imu_errors) &&
           (uint32_t)(now - g_last_imu_sample) <= S2_IMU_STALE_MS &&
           g_last_imu.motion_state != IMU_MOTION_LIFTED &&
           g_last_imu.motion_state != IMU_MOTION_FAULT;
}

static void pause_robot(MOTION_CONTROLLER * p_ctl, uint32_t now,
                        int32_t left_ticks, int32_t right_ticks)
{
    if (!g_b_armed)
    {
        g_b_start_requested =
            false; /* a PAUSE in Ready cancels a pending START */
        LOG_INFO(TAG_S2_ROBOT, "PAUSE in Ready: pending START cancelled");
        return;
    }
    if (g_b_halted && !g_b_line_halt)
    {
        LOG_WARN(TAG_S2_ROBOT, "PAUSE ignored: already in a safety stop");
        return;
    }
    vc_stop_motors(p_ctl, now, left_ticks, right_ticks);
    if (!g_b_paused)
    {
        s5_exec_set_following(&g_vc_s5, FALSE);
        (void)line_service_enable(false);
        g_b_turning  = false;
        g_turn_stage = 0u;
        g_b_paused   = true;
        LOG_INFO(TAG_S2_ROBOT, "paused (MQTT); RESUME to continue");
    }
}

static void resume_robot(uint32_t now)
{
    line_barcode_msg_t msg = {{0u, 0u, 0u}};
    if (!g_b_paused)
    {
        LOG_WARN(TAG_S2_ROBOT, "RESUME ignored: not paused%s",
                 g_b_armed ? "" : " (use START to leave Ready)");
        return;
    }
    if (!imu_healthy(now))
    {
        LOG_WARN(TAG_S2_ROBOT, "RESUME refused: IMU not healthy; still paused");
        return;
    }
    /* Same line test as the automatic re-arm. */
    if (line_service_snapshot(&msg) < E_OK ||
        (g_b_straddle ? (msg.data[1] & 7u) == LINE_BARCODE_FAULT
                      : (msg.data[0] & 3u) != 3u))
    {
        LOG_WARN(TAG_S2_ROBOT,
                 "RESUME refused: put the robot on the line; still paused");
        return;
    }
    g_b_paused = false;
    if (g_b_halted)
    {
        /* The recoverable line-stop path re-arms as usual, but the "held
           still" wait starts now: time spent paused does not count. */
        g_rearm_since = now;
        /* The operator put it back on the line: no black reading needed. */
        g_b_lost_seen_black = true;
        LOG_INFO(TAG_S2_ROBOT, "resumed; re-arms when the line is held still");
        return;
    }
    if (line_service_enable(true) < E_OK)
    {
        g_b_paused = true;
        LOG_ERR(TAG_S2_ROBOT, "RESUME failed: line service; still paused");
        return;
    }
    g_resumed_at           = now;
    g_b_line_unsafe_active = false;
    straddle_reset_steer();
    LOG_INFO(TAG_S2_ROBOT, "resumed (MQTT)");
}

/* Drain up to one queue's worth of MQTT commands without blocking. */
static void poll_commands(MOTION_CONTROLLER * p_ctl, uint32_t now,
                          int32_t left_ticks, int32_t right_ticks)
{
    for (unsigned idx = 0u; idx < S1_QUEUE_DEPTH; ++idx)
    {
        S1_COMMAND command;
        if (subsystem1_receive_command(&command, TMO_POL) < E_OK)
        {
            return;
        }
        switch (command.bytes[0])
        {
            case S1_COMMAND_STATUS: g_b_status_requested = true; break;
            case S1_COMMAND_PAUSE:
                pause_robot(p_ctl, now, left_ticks, right_ticks);
                break;
            case S1_COMMAND_RESUME: resume_robot(now); break;
            case S1_COMMAND_START:
                if (g_b_armed)
                {
                    LOG_WARN(TAG_S2_ROBOT, "START ignored: already started%s",
                             g_b_halted && !g_b_line_halt
                                 ? " (safety stop: reboot to retry)"
                             : g_b_paused ? " (paused: use RESUME)"
                                          : "");
                }
                else if (!g_b_start_requested)
                {
                    g_b_start_requested = true;
                    LOG_INFO(TAG_S2_ROBOT, "START received%s",
                             g_b_ready_reported
                                 ? ""
                                 : "; starts once initialisation finishes");
                }
                else
                {
                    /* No action needed in any other case. */
                }
                break;
            default: /* RESET_STATS, REVERSE: defined, not supported here */
                LOG_WARN(TAG_S2_ROBOT,
                         "command %u not supported by the integrated VC",
                         (unsigned)command.bytes[0]);
                break;
        }
    }
}

/* Apply the newest valid letter map from MQTT (S1 already validated it). */
static void poll_mapping(void)
{
    for (unsigned idx = 0u; idx < S1_QUEUE_DEPTH; ++idx)
    {
        S1_MAPPING    mapping;
        unsigned char letters[S1_MAP_SLOTS];
        if (subsystem1_receive_mapping(&mapping, TMO_POL) < E_OK)
        {
            return;
        }
        if (!s1_mapping_get(&mapping, letters))
        {
            continue;
        }
        if (line_service_set_mapping((char)('A' + letters[S1_MAP_LEFT]),
                                     (char)('A' + letters[S1_MAP_RIGHT]),
                                     (char)('A' + letters[S1_MAP_STRAIGHT]),
                                     (char)('A' + letters[S1_MAP_UTURN])) <
            E_OK)
        {
            LOG_WARN(TAG_S2_ROBOT, "MQTT map rejected by the line service");
            continue;
        }
        for (unsigned slot = 0u; slot < S1_MAP_SLOTS; ++slot)
        {
            g_map_letters[slot] = letters[slot];
        }
        g_b_map_received = true;
        LOG_INFO(TAG_S2_ROBOT,
                 "MQTT map applied: %c=left %c=right %c=straight %c=U-turn",
                 'A' + letters[0], 'A' + letters[1], 'A' + letters[2],
                 'A' + letters[3]);
    }
}

/* Telemetry direction for the last letter under the current map. */
static unsigned letter_direction(void)
{
    if (!g_b_map_received || g_last_letter == 0u)
    {
        return S1_BARCODE_NONE;
    }
    for (unsigned slot = 0u; slot < S1_MAP_SLOTS; ++slot)
    {
        if (g_map_letters[slot] == g_last_letter - 1u)
        {
            return S1_BARCODE_LEFT + slot;
        }
    }
    return S1_BARCODE_NONE;
}

static void update_distance(const MOTION_CONTROLLER * p_ctl, int32_t left_ticks,
                            int32_t right_ticks)
{
    if (g_b_odo_started && p_ctl->cfg.left_ticks_per_cm > 0.0f &&
        p_ctl->cfg.right_ticks_per_cm > 0.0f)
    {
        int32_t delta_left  = left_ticks - g_odo_l,
                delta_right = right_ticks - g_odo_r;
        if (delta_left < 0)
        {
            delta_left = -delta_left;
        }
        if (delta_right < 0)
        {
            delta_right = -delta_right;
        }
        g_distance_cm +=
            0.5f * ((float)delta_left / p_ctl->cfg.left_ticks_per_cm +
                    (float)delta_right / p_ctl->cfg.right_ticks_per_cm);
    }
    g_odo_l         = left_ticks;
    g_odo_r         = right_ticks;
    g_b_odo_started = true;
}

static unsigned robot_state(void)
{
    if (g_b_halted)
    {
        return g_b_line_halt ? S1_STATE_RECOVERING : S1_STATE_FAULT;
    }
    if (g_b_paused || !g_b_armed)
    {
        return S1_STATE_IDLE;
    }
    if (g_b_in_avoid)
    {
        return S1_STATE_AVOIDING;
    }
    return g_b_turning ? S1_STATE_TURNING : S1_STATE_FOLLOWING;
}

/* all_test's barcode tuning log: each new S3 frame's element widths, at
   most every 500 ms, so a misread can be diagnosed from the serial log. */
static uint32_t g_frame_report_ms, g_frame_sequence;

static void report_barcode_frame(uint32_t now)
{
    line_service_barcode_frame_t frame = {0};

    if ((uint32_t)(now - g_frame_report_ms) < 500u)
    {
        return;
    }
    g_frame_report_ms = now;
    line_service_barcode_frame(&frame);
    if (frame.sequence == g_frame_sequence)
    {
        return;
    }
    LOG_INFO(TAG_S2_BARCODE,
             "frame id=%lu runs=%u/29 black=%u white=%u result=%c skipped=%lu",
             (unsigned long)frame.sequence, (unsigned)frame.count,
             (unsigned)((frame.count + 1u) / 2u), (unsigned)(frame.count / 2u),
             frame.letter < 26u ? 'A' + frame.letter : '?',
             (unsigned long)(frame.sequence - g_frame_sequence - 1u));
    for (unsigned idx = 0; idx < frame.count; ++idx)
    {
        LOG_INFO(TAG_S2_BARCODE, "width frame=%lu run=%u %s width_ms=%u",
                 (unsigned long)frame.sequence, idx + 1u,
                 (idx & 1u) ? "WHITE" : "BLACK", (unsigned)frame.widths[idx]);
    }
    g_frame_sequence = frame.sequence;
}

/* IF-05 telemetry, once a second (or at once after STATUS). Nonblocking. */
static void report_telemetry(const MOTION_CONTROLLER * p_ctl, uint32_t now,
                             int32_t left_ticks, int32_t right_ticks)
{
    S1_TELEMETRY       packet;
    line_barcode_msg_t msg    = {{0u, 0u, 0u}};
    unsigned           faults = 0u, black = 0u, white = 0u, hump, sensors = 0u;
    uint8_t            debug_letter;
    S5_OBSTACLE_INFO   obstacle;
    float              speed;
    ER                 ercd;

    if (!g_b_status_requested && (uint32_t)(now - g_last_telemetry) < 1000u)
    {
        return;
    }
    g_b_status_requested = false;
    g_last_telemetry     = now;
    if (!imu_healthy(now))
    {
        faults |= S1_FAULT_IMU;
    }
    if (p_ctl->status.fault == MOTION_FAULT_LEFT_ENCODER_STALL ||
        p_ctl->status.fault == MOTION_FAULT_RIGHT_ENCODER_STALL)
    {
        faults |= S1_FAULT_MOTOR_STALL;
    }
    if (line_service_snapshot(&msg) >= E_OK)
    {
        unsigned s3_bits =
            msg.data[0] & 7u; /* S3: L/C/R bits 2/1/0; wire: 0/1/2 */
        sensors =
            ((s3_bits & 4u) >> 2u) | (s3_bits & 2u) | ((s3_bits & 1u) << 2u);
    }
    line_service_barcode_debug(&black, &white, &debug_letter);
    hump = g_b_have_imu ? g_last_imu.ramp_height_mm : 0u;
    speed =
        0.5f * (p_ctl->status.left_speed_cm_s + p_ctl->status.right_speed_cm_s);
    if (speed > (float)S1_SPEED_MAX)
    {
        speed = (float)S1_SPEED_MAX;
    }
    if (speed < (float)S1_SPEED_MIN)
    {
        speed = (float)S1_SPEED_MIN;
    }

    s1_telemetry_init(&packet);
    (void)s1_telemetry_set(&packet, S1_FIELD_STATE, (int)robot_state());
    (void)s1_telemetry_set(&packet, S1_FIELD_FAULTS, (int)faults);
    (void)s1_telemetry_set(&packet, S1_FIELD_SPEED_CMS, (int)speed);
    (void)s1_telemetry_set(&packet, S1_FIELD_SPEED_VALID, 1);
    (void)s1_telemetry_set(&packet, S1_FIELD_DISTANCE_CM,
                           g_distance_cm >= (float)S1_DISTANCE_MAX
                               ? S1_DISTANCE_MAX
                               : (int)g_distance_cm);
    (void)s1_telemetry_set(&packet, S1_FIELD_LINE_SENSORS, (int)sensors);
    (void)s1_telemetry_set(&packet, S1_FIELD_BARCODE, (int)letter_direction());
    (void)s1_telemetry_set(&packet, S1_FIELD_BARCODE_LETTER,
                           (int)g_last_letter);
    (void)s1_telemetry_set(&packet, S1_FIELD_MAP_RECEIVED,
                           g_b_map_received ? 1 : 0);
    /* #46: S5's safe-stop causes (CAL_FAIL, NO_PATH, LINE_LOST), latched until reboot */
    (void)s1_telemetry_set(&packet, S1_FIELD_S5_FAULTS,
                           (int)(subsystem5_get_faults() & S5_FAULT_MASK));
    (void)s1_telemetry_set(
        &packet, S1_FIELD_BARCODE_BLACK,
        black > S1_BARCODE_COUNT_MAX ? (int)S1_BARCODE_COUNT_MAX : (int)black);
    (void)s1_telemetry_set(
        &packet, S1_FIELD_BARCODE_WHITE,
        white > S1_BARCODE_COUNT_MAX ? (int)S1_BARCODE_COUNT_MAX : (int)white);
    (void)s1_telemetry_set(&packet, S1_FIELD_HIGHEST_HUMP_MM,
                           hump > S1_HUMP_MAX ? S1_HUMP_MAX : (int)hump);
    /* #41: low 16 bits of this step's raw counts (the ones distance uses;
       ctl->status lags a motion step). The PC takes int16 differences. */
    (void)s1_telemetry_set(&packet, S1_FIELD_LEFT_TICKS,
                           (int)((uint32_t)left_ticks & S1_TICKS_MASK));
    (void)s1_telemetry_set(&packet, S1_FIELD_RIGHT_TICKS,
                           (int)((uint32_t)right_ticks & S1_TICKS_MASK));
    /* #40: S5's last obstacle scan (all 0 until one found an obstacle; a
       corrupt word reads as 0 too). S5's ranges equal S1's. */
    if (subsystem5_get_obstacle(&obstacle) == E_OK)
    {
        (void)s1_telemetry_set(&packet, S1_FIELD_OBSTACLE_CM,
                               (int)obstacle.distance_cm);
        (void)s1_telemetry_set(&packet, S1_FIELD_SCAN_DEG,
                               (int)obstacle.angle_deg);
        (void)s1_telemetry_set(&packet, S1_FIELD_OBSTACLE_WIDTH_CM,
                               (int)obstacle.width_cm);
    }
    /* No battery field: nothing measures it (removed from the packet, #46). */
    ercd = subsystem1_publish_telemetry(&packet);
    if (ercd != g_telemetry_error && ercd != E_NOSPT)
    {
        LOG_INFO(TAG_S2_ROBOT, "telemetry queue %s (%d)",
                 ercd >= E_OK      ? "accepting packets again"
                 : ercd == E_TMOUT ? "full; waiting for MQTT"
                                   : "error",
                 (int)ercd);
    }
    g_telemetry_error = ercd;
}

/* ---- #8 straddle line model, ported from all_test's white_follow_step ----
   C and R sit either side of the line. S3's SEARCH/LOST/JUNCTION/COMMAND
   events assume the line under both sensors, so they are ignored here; S3
   faults still stop the robot, and S3 still decodes barcodes. */

/* Junction (C/R both black): a mapped stored letter drives onto the crossing,
   then pivots; otherwise go straight (decided 21 Nov). Returns true when a
   turn sequence started. */
static bool straddle_junction(MOTION_CONTROLLER * p_ctl, uint32_t now,
                              int32_t left_ticks, int32_t right_ticks)
{
    MOTION_COMMAND cmd;
    uint8_t        command;
    unsigned       letter    = g_pending_letter & LINE_BARCODE_LETTER_MASK;
    bool           b_waiting = (g_pending_letter & 0x80u) != 0u;

    g_pending_letter     = 0u;
    g_b_leaving_junction = true;
    g_junction_done      = now;
    g_steer_action       = 0u;
    g_b_clear_pending    = false;
    if (!b_waiting || line_service_lookup((uint8_t)letter, &command) != E_OK)
    {
        LOG_INFO(TAG_S2_ROBOT, "junction: %s, straight on",
                 !b_waiting         ? "no barcode"
                 : g_b_map_received ? "no mapped barcode"
                                    : "no MQTT letter map yet");
        return false;
    }
    if (line_service_enable(false) < E_OK)
    {
        halt(p_ctl, now, left_ticks, right_ticks, "line pause failed");
        return true;
    }
    /* all_test drives straight on from line following (JUNCTION profile);
       without profiles, stop first as the base moves need IDLE. */
    motion_control_set_profile(p_ctl, MOTION_PROFILE_JUNCTION);
    if (p_ctl->cfg.move_left_pwm_percent <= 0.0f)
    {
        vc_stop_motors(p_ctl, now, left_ticks, right_ticks);
        motion_control_step(p_ctl, now, left_ticks,
                            right_ticks); /* STOPPING -> IDLE */
    }
    cmd = (MOTION_COMMAND){MOTION_CMD_FORWARD_CM,
                           (command & 3u) == LINE_BARCODE_CMD_RIGHT
                               ? S2_TURN_ADVANCE_RIGHT_CM
                               : S2_TURN_ADVANCE_LEFT_CM};
    if (motion_control_command(p_ctl, &cmd, now, left_ticks, right_ticks) !=
        MOTION_RESULT_OK)
    {
        halt(p_ctl, now, left_ticks, right_ticks, "junction advance rejected");
        return true;
    }
    g_b_turning    = true;
    g_turn_started = now;
    g_turn_stage   = TURN_ADVANCE;
    g_turn_cmd     = command & 3u;
    LOG_INFO(TAG_S2_ROBOT, "junction: letter '%c' -> %s", 'A' + letter,
             g_turn_cmd == LINE_BARCODE_CMD_LEFT       ? "LEFT"
             : g_turn_cmd == LINE_BARCODE_CMD_RIGHT    ? "RIGHT"
             : g_turn_cmd == LINE_BARCODE_CMD_STRAIGHT ? "STRAIGHT"
                                                       : "U-TURN");
    return true;
}

/* Distance in mm since the lost-line watch began (mean of both wheels). */
static uint32_t lost_watch_mm(int32_t left_ticks, int32_t right_ticks)
{
    float dist_cm =
        0.5f * ((float)(left_ticks - g_lost_l0) / S2_ENC_LEFT_TICKS_PER_CM +
                (float)(right_ticks - g_lost_r0) / S2_ENC_RIGHT_TICKS_PER_CM);
    return dist_cm > 0.0f ? (uint32_t)(dist_cm * 10.0f) : 0u;
}

/* #23: detect a lost line in the straddle model (see S2_LINE_LOST_MM).
   Call with this step's C/R bits before the steering state is updated, so
   the first white sample after a correction still sees g_b_clear_pending false.
   After S2_LINE_LOST_MM of white the robot first probes: it steers softly
   toward the side the tape was last under for up to S2_LINE_PROBE_MM.
   Tape left between C and R (robot running parallel to it, 23 Nov) then
   passes under the other sensor; tape just outside the last sensor passes
   back under it. Either way the watch ends and following carries on. Only
   no black at all through the probe stops the robot. Returns true when it
   stopped the robot; *probe_steer is the steer to command while probing,
   0 when not probing. */
static bool line_lost_check(MOTION_CONTROLLER * p_ctl, uint32_t now,
                            int32_t left_ticks, int32_t right_ticks,
                            unsigned sensors, float * p_probe_steer)
{
    *p_probe_steer = 0.0f;
#if S2_LINE_LOST_MM > 0
    /* Not following for a while (turn, avoidance, pause): the start ticks
       are stale. */
    if ((uint32_t)(now - g_lost_last_step) > S2_LINE_LOST_GAP_MS)
    {
        g_b_lost_watch = g_b_lost_probe = false;
    }
    g_lost_last_step = now;
    if (sensors != 0u)
    {
        if (g_b_lost_watch)
        {
            uint32_t dist_mm = lost_watch_mm(left_ticks, right_ticks);
            if (dist_mm > g_lost_max_mm)
            {
                g_lost_max_mm = dist_mm;
            }
            if (g_b_lost_probe)
            {
                LOG_INFO(TAG_S2_ROBOT,
                         "line probe found the tape after %u mm; following",
                         (unsigned)dist_mm);
            }
        }
        g_b_lost_watch = g_b_lost_probe = false;
        return false;
    }
    if (!g_b_lost_watch)
    {
        /* First white sample after a long enough correction. */
        if (g_steer_action != 0u && !g_b_clear_pending &&
            (uint32_t)(now - g_action_since) >= S2_LINE_LOST_ARM_MS)
        {
            g_b_lost_watch = true;
            g_lost_side    = g_steer_action;
            g_lost_l0      = left_ticks;
            g_lost_r0      = right_ticks;
        }
        return false;
    }
    uint32_t dist_mm = lost_watch_mm(left_ticks, right_ticks);
    if (dist_mm < S2_LINE_LOST_MM)
    {
        return false;
    }
    if (!g_b_lost_probe && S2_LINE_PROBE_MM > 0u)
    {
        g_b_lost_probe = true;
        LOG_INFO(
            TAG_S2_ROBOT,
            "line probe: no black for %u mm after a %s correction; "
            "steering %s up to %u mm",
            (unsigned)S2_LINE_LOST_MM, g_lost_side == 1u ? "right" : "left",
            g_lost_side == 1u ? "right" : "left", (unsigned)S2_LINE_PROBE_MM);
    }
    if (dist_mm < S2_LINE_LOST_MM + S2_LINE_PROBE_MM)
    {
        /* g_lost_side 1 = R was black (steer right, +); 2 = C (left, -). */
        *p_probe_steer =
            g_lost_side == 1u ? S2_LINE_PROBE_STEER : -S2_LINE_PROBE_STEER;
        return false;
    }
    LOG_WARN(TAG_S2_ROBOT,
             "line lost: no black on C or R for %u mm after a %s correction "
             "(probe included)",
             (unsigned)dist_mm, g_lost_side == 1u ? "right" : "left");
    g_b_lost_watch = g_b_lost_probe = false;
    halt_line(p_ctl, now, left_ticks, right_ticks,
              "line lost; put the tape under C or R, or PAUSE + RESUME");
    g_b_lost_stop       = true;
    g_b_lost_seen_black = false;
    return true;
#else
    (void)p_ctl;
    (void)now;
    (void)left_ticks;
    (void)right_ticks;
    (void)sensors;
    return false;
#endif
}

/* Forget any correction in progress: after a pause or a line stop the robot
   may have been moved by hand, so an old correction (and its hard-correction
   timer) must not carry over (#23). */
static void straddle_reset_steer(void)
{
    g_steer_action = g_steer_candidate = 0u;
    g_b_clear_pending                  = false;
    g_b_lost_watch = g_b_lost_probe = false;
}

static void straddle_step(MOTION_CONTROLLER * p_ctl, uint32_t now,
                          int32_t left_ticks, int32_t right_ticks)
{
    line_barcode_msg_t msg = {{0u, 0u, 0u}};
    MOTION_COMMAND     cmd;
    ER                 ercd  = line_service_snapshot(&msg);
    unsigned           state = msg.data[1] & 7u, sensors;

    /* S3 faults stop at once; a missing or stuck-paused snapshot only after
       S2_LINE_UNSAFE_MS, as in the S3 model. */
    if (ercd >= E_OK && state == LINE_BARCODE_FAULT)
    {
        halt_line(p_ctl, now, left_ticks, right_ticks, "line sensor fault");
        return;
    }
    if (ercd < E_OK || state == LINE_BARCODE_PAUSED)
    {
        if (!g_b_line_unsafe_active)
        {
            g_b_line_unsafe_active = true;
            g_line_unsafe_since    = now;
        }
        if ((uint32_t)(now - g_line_unsafe_since) >= S2_LINE_UNSAFE_MS)
        {
            halt_line(p_ctl, now, left_ticks, right_ticks,
                      "line snapshot unsafe or stale");
        }
        return;
    }
    g_b_line_unsafe_active = false;
    /* all_test slows down while GP28 is on a bar or a capture is open. */
    motion_control_set_scanning(
        p_ctl, S2_LINE_LAYOUT == LINE_BARCODE_LAYOUT_SIDE &&
                   ((msg.data[0] & LINE_BARCODE_LEFT) != 0u ||
                    (msg.data[1] & LINE_BARCODE_CAPTURE_ACTIVE) != 0u));

    /* Barcodes: remember the newest letter until the next junction. */
    for (unsigned idx = 0; idx < 24u; ++idx)
    {
        line_barcode_msg_t frame;
        ercd = line_service_receive(&frame, TMO_POL);
        if (ercd == E_TMOUT)
        {
            break;
        }
        if (ercd < E_OK)
        {
            halt(p_ctl, now, left_ticks, right_ticks, "line receive failed");
            return;
        }
        unsigned event = (frame.data[1] >> 3u) & 7u;
        if (event == LINE_BARCODE_EVT_FAULT)
        {
            halt_line(p_ctl, now, left_ticks, right_ticks, "line sensor fault");
            return;
        }
        if (event == LINE_BARCODE_EVT_INVALID)
        {
            LOG_WARN(TAG_S2_BARCODE,
                     "rejected invalid frame: check start/stop, narrow/wide "
                     "widths and white margins");
        }
        if (event == LINE_BARCODE_EVT_BARCODE)
        {
            unsigned letter = frame.data[2] & LINE_BARCODE_LETTER_MASK;
            uint8_t  command;
            g_pending_letter = (uint8_t)(0x80u | letter);
            g_last_letter    = letter + 1u;
            LOG_INFO(TAG_S2_BARCODE,
                     "decoded=%c queued=%s; execute at next C/R junction",
                     'A' + letter,
                     line_service_lookup((uint8_t)letter, &command) != E_OK
                         ? "no mapping"
                     : command == LINE_BARCODE_CMD_LEFT     ? "LEFT"
                     : command == LINE_BARCODE_CMD_RIGHT    ? "RIGHT"
                     : command == LINE_BARCODE_CMD_STRAIGHT ? "STRAIGHT"
                                                            : "U-TURN");
        }
    }

    sensors = msg.data[0] & 3u; /* C bit 1, R bit 0; the scanner is ignored */
    float probe_steer;
    if (line_lost_check(p_ctl, now, left_ticks, right_ticks, sensors,
                        &probe_steer))
    {
        return;
    }
    if (g_b_leaving_junction && sensors != 3u &&
        (uint32_t)(now - g_junction_done) >= S2_JUNCTION_SETTLE_MS)
    {
        g_b_leaving_junction = false;
    }
    if (sensors == 3u && !g_b_leaving_junction &&
        straddle_junction(p_ctl, now, left_ticks, right_ticks))
    {
        return;
    }

    if (sensors == 1u || sensors == 2u)
    {
        if (sensors != g_steer_candidate)
        {
            g_steer_candidate = sensors;
            g_candidate_since = now;
        }
        /* A first correction acts at once; only a reversal must persist. */
        if (g_steer_action == 0u ||
            (uint32_t)(now - g_candidate_since) >= S2_LINE_CONFIRM_MS)
        {
            if (g_steer_action != sensors)
            {
                g_action_since = now;
            }
            g_steer_action = sensors;
        }
        g_b_clear_pending = false;
    }
    else if (sensors == 0u && g_steer_action != 0u)
    {
        g_steer_candidate = 0u;
        if (!g_b_clear_pending)
        {
            g_b_clear_pending = true;
            g_clear_since     = now;
        }
        if ((uint32_t)(now - g_clear_since) >= S2_WHITE_CLEAR_MS)
        {
            g_steer_action    = 0u;
            g_b_clear_pending = false;
        }
    }
    else
    {
        /* No action needed in any other case. */
    }
    if (sensors == 0u)
    {
        g_steer_candidate = 0u;
    }

    /* S2-U09 diagnosis (22 Nov): once a second, how many control steps each
       sensor pattern and each steering action lasted. sensors: 0 neither
       black, 1 R only (drifting left, steer right), 2 C only (drifting
       right, steer left), 3 both; act: 0 straight, 1 right, 2 left. */
    {
        static uint32_t log_t0, log_n;
        static uint16_t sens_n[4], act_n[3];
        if (log_n == 0u)
        {
            log_t0 = now;
        }
        log_n++;
        sens_n[sensors & 3u]++;
        act_n[g_steer_action > 2u ? 0u : g_steer_action]++;
        if ((uint32_t)(now - log_t0) >= 1000u)
        {
            /* watch max: longest white run (mm) after a correction that
               ended in black again; tunes S2_LINE_LOST_MM (#23). bias: the
               learned straight-trim I term in tenths of a PWM point,
               + = left wheel boosted (S2_FOLLOW_STRAIGHT_KI). */
            LOG_INFO(TAG_S2_ROBOT,
                     "steer n=%u sens none=%u R=%u C=%u both=%u act "
                     "straight=%u right=%u left=%u watch max=%umm bias=%d",
                     (unsigned)log_n, sens_n[0], sens_n[1], sens_n[2],
                     sens_n[3], act_n[0], act_n[1], act_n[2],
                     (unsigned)g_lost_max_mm,
                     (int)(p_ctl->straight_bias * 10.0f));
            log_n         = 0u;
            g_lost_max_mm = 0u;
            for (unsigned idx = 0; idx < 4u; ++idx)
            {
                sens_n[idx] = 0u;
            }
            for (unsigned idx = 0; idx < 3u; ++idx)
            {
                act_n[idx] = 0u;
            }
        }
    }

    /* Right sensor black: drifting left, steer right (positive). */
    float strength =
        (uint32_t)(now - g_action_since) >= S2_LINE_HARD_MS ? 1.0f : 0.5f;
    cmd.type  = MOTION_CMD_FOLLOW_LINE;
    cmd.value = probe_steer != 0.0f ? probe_steer : /* #23 lost-line probe */
                    sensors == 3u ? 0.0f
                                  : /* on the crossing: straight */
                    g_steer_action == 1u ? strength
                : g_steer_action == 2u   ? -strength
                                         : 0.0f;
    if (motion_control_command(p_ctl, &cmd, now, left_ticks, right_ticks) !=
        MOTION_RESULT_OK)
    {
        halt(p_ctl, now, left_ticks, right_ticks, "follow command rejected");
    }
}

static void control_step(MOTION_CONTROLLER * p_ctl, uint32_t now,
                         int32_t left_ticks, int32_t right_ticks)
{
    IMU_DATA           imu;
    line_barcode_msg_t msg = {{0u, 0u, 0u}};
    MOTION_COMMAND     cmd;
    ER                 ercd;

    g_b_in_avoid = false;
    if (g_b_halted)
    {
        /* PAUSE also blocks the automatic line re-arm. */
        if (!g_b_line_halt || g_b_paused ||
            !line_rearm_ready(now, left_ticks, right_ticks))
        {
            cmd = (MOTION_COMMAND){MOTION_CMD_STOP, 0.0f};
            (void)motion_control_command(p_ctl, &cmd, now, left_ticks,
                                         right_ticks);
            return;
        }
        g_b_halted    = false;
        g_b_line_halt = false;
        g_b_turning   = false;
        ercd          = line_service_enable(true);
        if (ercd < E_OK)
        {
            halt(p_ctl, now, left_ticks, right_ticks, "line resume failed");
            return;
        }
        g_resumed_at           = now;
        g_b_line_unsafe_active = false;
        g_b_lost_stop          = false;
        straddle_reset_steer();
        LOG_INFO(TAG_S2_ROBOT, "line found, re-armed");
        return;
    }
    if (g_b_paused)
    {
        vc_stop_motors(p_ctl, now, left_ticks, right_ticks);
        return;
    }
    if (p_ctl->status.state == MOTION_STATE_FAULT)
    {
        halt(p_ctl, now, left_ticks, right_ticks,
             "motion fault / calibration missing");
        return;
    }
    /* robot_control_step() reads S4 once per step into g_last_imu. */
    if (!g_b_have_imu)
    {
        halt(p_ctl, now, left_ticks, right_ticks, "IMU snapshot failed");
        return;
    }
    imu = g_last_imu;
    if (!g_b_armed)
    {
        /* Remain stationary throughout IMU baseline calibration, and while
           S5 holds at start-up (its servo swing shakes the IMU). */
        if (s5_exec_startup_hold())
        {
            return;
        }
        if (!S2_LINE_CALIBRATION_READY || now < S2_ROBOT_START_MS ||
            !imu.sensor_ok || !imu.calibrated || g_imu_count == 0 ||
            (uint32_t)(now - g_last_imu_sample) > S2_IMU_STALE_MS)
        {
            return;
        }
        /* Ready (#48): initialised, motors off, waiting for START. */
        if (!g_b_auto_start && !g_b_start_requested)
        {
            if (!g_b_ready_reported)
            {
                g_b_ready_reported = true;
                LOG_INFO(TAG_S2_ROBOT,
                         "ready: motors off, waiting for START over MQTT");
            }
            return;
        }
        if (!g_b_configured)
        {
            ercd = line_service_calibrate(S2_LINE_BLACK_RAW, S2_LINE_WHITE_RAW);
            if (ercd == E_OK)
            {
                ercd = line_service_set_layout(S2_LINE_LAYOUT);
            }
            /* No built-in letter map (#72): the map arrives over MQTT. */
            if (ercd == E_OK)
            {
                ercd = line_service_enable(true);
            }
            if (ercd < E_OK)
            {
                halt(p_ctl, now, left_ticks, right_ticks,
                     "line configuration failed");
                return;
            }
            g_b_configured = true;
            g_resumed_at   = now;
            g_imu_errors   = imu.read_errors;
            g_b_armed      = true;
            LOG_INFO(TAG_S2_ROBOT, "armed: %s",
                     g_b_map_received
                         ? "letter map from MQTT"
                         : "no letter map yet, straight at junctions");
        }
    }
    /* Tilt is report-only (#67, decided 20 Nov 2026): S4 still logs it and
       S5 ignores ultrasonic readings while tilted, but it never stops the
       robot. A false tilt would otherwise latch a stop until reboot.
       Lifting the robot (#53, replaces collision) stops it and the stop
       latches until reboot, like every IMU stop. */
    bool b_lifted =
        imu.motion_state == IMU_MOTION_LIFTED || imu.event == IMU_EVENT_LIFT;
    if (b_lifted || !imu.sensor_ok || !imu.calibrated ||
        imu.read_errors != g_imu_errors ||
        (uint32_t)(now - g_last_imu_sample) > S2_IMU_STALE_MS ||
        imu.motion_state == IMU_MOTION_FAULT || imu.event == IMU_EVENT_FAULT)
    {
        halt(p_ctl, now, left_ticks, right_ticks,
             b_lifted ? "robot lifted (IMU)" : "IMU unsafe or stale");
        /* all_test: the values that latched the stop, printed once. */
        LOG_INFO(TAG_S2_ROBOT,
                 "IMU stop data: age=%lu ms samples=%lu ok=%d calibrated=%d "
                 "motion=%d event=%d errors=%lu baseline_errors=%lu",
                 (unsigned long)(now - g_last_imu_sample),
                 (unsigned long)g_imu_count, (int)imu.sensor_ok,
                 (int)imu.calibrated, (int)imu.motion_state, (int)imu.event,
                 (unsigned long)imu.read_errors, (unsigned long)g_imu_errors);
        return;
    }
    /* S5 obstacle avoidance takes control while it has a request. */
    g_b_in_avoid = avoid_step(p_ctl, now, left_ticks, right_ticks);
    if (g_b_in_avoid)
    {
        return;
    }
    /* Hump is terrain information; keep following, without inventing speed
       compensation before physical tuning. */
    if (g_b_turning)
    {
        bool b_uturn = p_ctl->cfg.move_left_pwm_percent > 0.0f &&
                       g_turn_stage == TURN_PIVOT &&
                       g_turn_cmd == LINE_BARCODE_CMD_UTURN;

        if (b_uturn)
        {
            /* all_test: the counter-rotating U-turn ends once the right wheel
               has covered half its 180-degree pivot arc AND the centre
               sensor is back on black. */
            int32_t moved = right_ticks - g_uturn_r0;
            float   target =
                180.0f * p_ctl->cfg.right_turn_ticks_per_degree * 0.5f;
            if (moved < 0)
            {
                moved = -moved;
            }
            if ((float)moved >= target && line_service_snapshot(&msg) >= E_OK &&
                (msg.data[0] & LINE_BARCODE_CENTRE) != 0u)
            {
                vc_stop_motors(p_ctl, now, left_ticks, right_ticks);
                motion_control_step(p_ctl, now, left_ticks,
                                    right_ticks); /* STOPPING -> IDLE */
            }
        }
        /* 22 Nov: a left/right pivot that is past S2_PIVOT_LINE_MIN_FRACTION of
           its arc stops as soon as the centre sensor sees the line, instead of
           turning on past it to the full tick count. Without a line it still
           ends on the full count. */
        if (p_ctl->cfg.move_left_pwm_percent > 0.0f &&
            g_turn_stage == TURN_PIVOT &&
            (g_turn_cmd == LINE_BARCODE_CMD_LEFT ||
             g_turn_cmd == LINE_BARCODE_CMD_RIGHT) &&
            p_ctl->status.state != MOTION_STATE_IDLE &&
            p_ctl->status.state != MOTION_STATE_STOPPING)
        {
            bool b_left =
                g_turn_cmd ==
                LINE_BARCODE_CMD_LEFT; /* left pivot: right wheel drives */
            int32_t moved =
                b_left ? right_ticks - g_pivot_r0 : left_ticks - g_pivot_l0;
            float arc =
                90.0f * (b_left ? p_ctl->cfg.right_turn_ticks_per_degree
                                : p_ctl->cfg.left_turn_ticks_per_degree);
            if (moved < 0)
            {
                moved = -moved;
            }
            if ((float)moved >= arc * (b_left ? S2_PIVOT_LINE_MIN_FRACTION_LEFT
                                              : S2_PIVOT_LINE_MIN_FRACTION) &&
                line_service_snapshot(&msg) >= E_OK &&
                (msg.data[0] & LINE_BARCODE_CENTRE) != 0u)
            {
                LOG_INFO(TAG_S2_ROBOT,
                         "pivot stopped on the line at %d%% of the arc",
                         (int)(100.0f * (float)moved / arc));
                vc_stop_motors(p_ctl, now, left_ticks, right_ticks);
                motion_control_step(p_ctl, now, left_ticks,
                                    right_ticks); /* STOPPING -> IDLE */
            }
        }
        if ((uint32_t)(now - g_turn_started) > S2_ROBOT_TURN_TIMEOUT_MS)
        {
            halt(p_ctl, now, left_ticks, right_ticks, "turn timeout");
        }
        else if (p_ctl->status.state == MOTION_STATE_IDLE &&
                 p_ctl->status.motion_complete)
        {
            if (g_turn_stage == TURN_ADVANCE &&
                g_turn_cmd != LINE_BARCODE_CMD_STRAIGHT)
            {
                /* #8: on the crossing now; pivot as all_test does. */
                cmd.type = g_turn_cmd == LINE_BARCODE_CMD_RIGHT
                               ? MOTION_CMD_TURN_RIGHT_DEG
                               : MOTION_CMD_TURN_LEFT_DEG;
                cmd.value =
                    g_turn_cmd == LINE_BARCODE_CMD_UTURN ? 180.0f : 90.0f;
                motion_control_set_profile(p_ctl,
                                           g_turn_cmd == LINE_BARCODE_CMD_UTURN
                                               ? MOTION_PROFILE_UTURN
                                               : MOTION_PROFILE_JUNCTION);
                g_turn_stage = TURN_PIVOT;
                g_uturn_r0   = right_ticks;
                g_pivot_l0   = left_ticks;
                g_pivot_r0   = right_ticks;
                if (motion_control_command(p_ctl, &cmd, now, left_ticks,
                                           right_ticks) != MOTION_RESULT_OK)
                {
                    halt(p_ctl, now, left_ticks, right_ticks, "turn rejected");
                }
                return;
            }
            g_b_turning            = false;
            g_turn_stage           = 0u;
            ercd                   = line_service_enable(true);
            g_resumed_at           = now;
            g_b_line_unsafe_active = false;
            /* #8: the sensors may still be on the crossing just handled. */
            g_b_leaving_junction = g_b_straddle;
            g_junction_done      = now;
            g_steer_action       = 0u;
            if (ercd < E_OK)
            {
                halt(p_ctl, now, left_ticks, right_ticks, "line resume failed");
            }
        }
        else
        {
            /* No action needed in any other case. */
        }
        return;
    }
    /* Permit two sensor periods after enable/reset, with zero motor output. */
    if ((uint32_t)(now - g_resumed_at) < 30u)
    {
        return;
    }
    if (g_b_straddle)
    {
        straddle_step(p_ctl, now, left_ticks, right_ticks);
        return;
    }

    /* A newer snapshot fault must win over an older queued turn command. */
    ercd = line_service_snapshot(&msg);
    if (!line_snapshot_ok(p_ctl, now, left_ticks, right_ticks, ercd, &msg,
                          false))
    {
        return;
    }

    /* Drain events with a fixed budget so control cannot starve. */
    for (unsigned idx = 0; idx < 24u; ++idx)
    {
        ercd = line_service_receive(&msg, TMO_POL);
        if (ercd == E_TMOUT)
        {
            break;
        }
        if (ercd < E_OK)
        {
            halt(p_ctl, now, left_ticks, right_ticks, "line receive failed");
            return;
        }
        uint8_t event = (msg.data[1] >> 3u) & 7u;
        if (event == LINE_BARCODE_EVT_BARCODE ||
            event == LINE_BARCODE_EVT_COMMAND)
        {
            g_last_letter = (msg.data[2] & LINE_BARCODE_LETTER_MASK) + 1u;
        }
        if (event == LINE_BARCODE_EVT_JUNCTION)
        {
            /* #72 (decided 21 Nov): a junction without a mapped barcode before
               it goes straight, with or without an MQTT map (as all_test). */
            LOG_INFO(TAG_S2_ROBOT, "junction: %s, straight on",
                     g_b_map_received ? "no mapped barcode"
                                      : "no MQTT letter map yet");
            continue;
        }
        if (event == LINE_BARCODE_EVT_LOST || event == LINE_BARCODE_EVT_FAULT)
        {
            halt_line(p_ctl, now, left_ticks, right_ticks,
                      "line lost or fault");
            return;
        }
        if (event == LINE_BARCODE_EVT_COMMAND)
        {
            unsigned action = msg.data[0] >> 6u;
            if (action == LINE_BARCODE_CMD_STRAIGHT)
            {
                continue;
            }
            ercd = line_service_enable(false);
            if (ercd < E_OK)
            {
                halt(p_ctl, now, left_ticks, right_ticks, "line pause failed");
                return;
            }
            cmd = (MOTION_COMMAND){MOTION_CMD_STOP, 0.0f};
            (void)motion_control_command(p_ctl, &cmd, now, left_ticks,
                                         right_ticks);
            motion_control_step(p_ctl, now, left_ticks,
                                right_ticks); /* STOPPING -> IDLE */
            cmd.type  = action == LINE_BARCODE_CMD_RIGHT
                            ? MOTION_CMD_TURN_RIGHT_DEG
                            : MOTION_CMD_TURN_LEFT_DEG;
            cmd.value = action == LINE_BARCODE_CMD_UTURN ? 180.0f : 90.0f;
            motion_control_set_profile(p_ctl,
                                       MOTION_PROFILE_BASE); /* S3 model */
            if (motion_control_command(p_ctl, &cmd, now, left_ticks,
                                       right_ticks) != MOTION_RESULT_OK)
            {
                halt(p_ctl, now, left_ticks, right_ticks, "turn rejected");
                return;
            }
            g_b_turning    = true;
            g_turn_started = now;
            return;
        }
    }
    ercd = line_service_snapshot(&msg);
    if (!line_snapshot_ok(p_ctl, now, left_ticks, right_ticks, ercd, &msg,
                          true))
    {
        return;
    }
    unsigned pos = (msg.data[0] >> 3u) & 7u;
    cmd.type     = MOTION_CMD_FOLLOW_LINE;
    /* S3's positions increase from far-left to far-right. */
    cmd.value = pos <= 4u ? ((float)pos - 2.0f) * 0.5f : 0.0f;
    if (motion_control_command(p_ctl, &cmd, now, left_ticks, right_ticks) !=
        MOTION_RESULT_OK)
    {
        halt(p_ctl, now, left_ticks, right_ticks, "follow command rejected");
    }
}

void robot_control_step(MOTION_CONTROLLER * p_ctl, uint32_t now,
                        int32_t left_ticks, int32_t right_ticks)
{
    IMU_DATA imu;
    /* One S4 read per step, shared by control, RESUME checks and telemetry. */
    g_b_have_imu = subsystem4_get_latest(&imu) >= E_OK;
    if (g_b_have_imu)
    {
        g_last_imu = imu;
        if (imu.sample_count != g_imu_count)
        {
            g_imu_count       = imu.sample_count;
            g_last_imu_sample = now;
        }
    }
    poll_mapping();
    poll_commands(p_ctl, now, left_ticks, right_ticks);
    control_step(p_ctl, now, left_ticks, right_ticks);
    update_distance(p_ctl, left_ticks, right_ticks);
    report_telemetry(p_ctl, now, left_ticks, right_ticks);
    report_barcode_frame(now);
}

#elif S2_MOTION_DEMO && !APP_TEST

#if S2_PID_BENCH
volatile uint32_t g_s2_bench_stop_ms; /* set when run 2 sends STOP */
/*
 * S2-U10 / S2-U11 floor bench (#18). Two 100 cm forward runs, 5 s apart.
 *   run 1 (U10): logs left-minus-right travel and PWM every 20 ms, so the
 *                straight-line PID's response can be read from the console.
 *   run 2 (U11): sends STOP through subsystem2_send_command() (the same path
 *                as the real STOP) S2_BENCH_STOP_AFTER_MS into the run.
 *                task_motion.c times the motors-off and wheels-still points.
 * Needs 1.2 m of clear floor ahead. Not part of any normal build.
 */
void robot_control_step(MOTION_CONTROLLER * p_ctl, uint32_t now,
                        int32_t left_ticks, int32_t right_ticks)
{
    static const MOTION_COMMAND fwd = {MOTION_CMD_FORWARD_CM, 100.0f};
    static unsigned             run; /* 0, 1: finished runs */
    static bool                 b_active, b_stop_sent, b_done;
    static uint32_t             started, next_start, step_no;
    static bool                 b_first = true;
    MOTION_COMMAND              stop    = {MOTION_CMD_STOP, 0.0f};

    if (b_first)
    { /* the kernel clock does not start at 0: count from here */
        b_first    = false;
        next_start = now + S2_ROBOT_START_MS;
    }
    if (b_done || p_ctl->status.state == MOTION_STATE_FAULT)
    {
        return;
    }
    if (!b_active)
    {
        if ((int32_t)(now - next_start) >= 0 &&
            motion_control_command(p_ctl, &fwd, now, left_ticks, right_ticks) ==
                MOTION_RESULT_OK)
        {
            b_active    = true;
            b_stop_sent = false;
            started     = now;
            step_no     = 0u;
            LOG_INFO(TAG_S2_BENCH,
                     "run %u (%s) start; cols: ms L R err_mm pwmL pwmR",
                     run + 1u, run == 0u ? "U10 PID" : "U11 STOP");
        }
        return;
    }
    {
        uint32_t elapsed_ms = now - started;
        if ((step_no++ & 1u) == 0u)
        {
            /* err > 0: the left wheel is ahead of the right. */
            float left_cm =
                (float)(left_ticks - p_ctl->command_start_left_ticks) /
                p_ctl->cfg.left_ticks_per_cm;
            float right_cm =
                (float)(right_ticks - p_ctl->command_start_right_ticks) /
                p_ctl->cfg.right_ticks_per_cm;
            LOG_INFO(TAG_S2_BENCH, "%lu %ld %ld %ld %d %d",
                     (unsigned long)elapsed_ms, (long)left_ticks,
                     (long)right_ticks, (long)((left_cm - right_cm) * 10.0f),
                     (int)p_ctl->status.left_pwm_percent,
                     (int)p_ctl->status.right_pwm_percent);
        }
        if (run == 1u && !b_stop_sent && elapsed_ms >= S2_BENCH_STOP_AFTER_MS)
        {
            b_stop_sent = true;
            LOG_INFO(TAG_S2_BENCH, "STOP sent at %lu ms",
                     (unsigned long)elapsed_ms);
            g_s2_bench_stop_ms = now ? now : 1u;
            (void)subsystem2_send_command(&stop);
        }
        if (p_ctl->status.state == MOTION_STATE_IDLE ||
            elapsed_ms > S2_DEMO_TIMEOUT_MS ||
            (b_stop_sent && elapsed_ms > S2_BENCH_STOP_AFTER_MS + 500u))
        {
            LOG_INFO(TAG_S2_BENCH, "run %u end at %lu ms, state=%d", run + 1u,
                     (unsigned long)elapsed_ms, (int)p_ctl->status.state);
            b_active   = false;
            next_start = now + 5000u;
            if (++run == 2u)
            {
                b_done = true;
                (void)motion_control_command(p_ctl, &stop, now, left_ticks,
                                             right_ticks);
                LOG_INFO(TAG_S2_BENCH, "finished; motors OFF");
            }
        }
    }
}
#elif S2_PIVOT_BENCH
/*
 * S2-U07 floor bench (#18): motor-driven one-wheel pivots at S2_PIVOT_BENCH_PWM,
 * 8 s apart so the real heading can be marked on the floor each time. Order:
 * right 90, right 90, left 90, left 90, right 360, left 360. After each turn it
 * logs the encoder ticks since the turn began; ticks/degree is then
 * (moving wheel ticks) / (measured degrees).
 */
void robot_control_step(MOTION_CONTROLLER * p_ctl, uint32_t now,
                        int32_t left_ticks, int32_t right_ticks)
{
    static const MOTION_COMMAND seq[] = {{MOTION_CMD_TURN_RIGHT_DEG, 90.0f},
                                         {MOTION_CMD_TURN_RIGHT_DEG, 90.0f},
                                         {MOTION_CMD_TURN_LEFT_DEG, 90.0f},
                                         {MOTION_CMD_TURN_LEFT_DEG, 90.0f},
                                         {MOTION_CMD_TURN_RIGHT_DEG, 360.0f},
                                         {MOTION_CMD_TURN_LEFT_DEG, 360.0f}};
    static unsigned             stage;
    static bool                 b_waiting, b_done;
    static uint32_t             started, pause_until, last_warn, last_beat;
    static bool                 b_banner;
    MOTION_COMMAND              stop = {MOTION_CMD_STOP, 0.0f};

    if (!b_banner)
    {
        b_banner = true;
        /* tk_get_tim() does not start at 0 at boot (it read 3.35e9 ms), so
           the start delay counts from the first call, not from time 0. */
        pause_until = now + S2_ROBOT_START_MS;
        LOG_INFO(TAG_S2_BENCH, "pivot bench: first turn in %u ms, state=%d",
                 (UINT)S2_ROBOT_START_MS, (int)p_ctl->status.state);
    }
    if ((uint32_t)(now - last_beat) >= 3000u)
    {
        last_beat = now;
        LOG_INFO(TAG_S2_BENCH,
                 "beat now=%lu next=%lu waiting=%d done=%d stage=%u state=%d",
                 (unsigned long)now, (unsigned long)pause_until, (int)b_waiting,
                 (int)b_done, stage, (int)p_ctl->status.state);
    }
    if (b_done)
    {
        (void)motion_control_command(p_ctl, &stop, now, left_ticks,
                                     right_ticks);
        return;
    }
    if (p_ctl->status.state == MOTION_STATE_FAULT)
    {
        return;
    }
    if (b_waiting)
    {
        if (p_ctl->status.state == MOTION_STATE_IDLE &&
            p_ctl->status.motion_complete)
        {
            LOG_INFO(TAG_S2_BENCH,
                     "turn %u (%s %d) done; ticks since start dL=%ld dR=%ld. "
                     "Mark the heading now",
                     stage + 1u,
                     seq[stage].type == MOTION_CMD_TURN_RIGHT_DEG ? "right"
                                                                  : "left",
                     (int)seq[stage].value,
                     (long)(left_ticks - p_ctl->command_start_left_ticks),
                     (long)(right_ticks - p_ctl->command_start_right_ticks));
            b_waiting   = false;
            pause_until = now + 8000u;
            if (++stage == sizeof(seq) / sizeof(seq[0]))
            {
                b_done = true;
                LOG_INFO(TAG_S2_BENCH, "finished; motors OFF");
            }
        }
        else if ((uint32_t)(now - started) > S2_DEMO_TIMEOUT_MS)
        {
            (void)motion_control_command(p_ctl, &stop, now, left_ticks,
                                         right_ticks);
            b_done = true;
            LOG_WARN(TAG_S2_BENCH, "timeout; motors OFF");
        }
        else
        {
            /* No action needed in any other case. */
        }
    }
    else if ((int32_t)(now - pause_until) >= 0)
    {
        MOTION_RESULT res = motion_control_command(p_ctl, &seq[stage], now,
                                                   left_ticks, right_ticks);
        if (res == MOTION_RESULT_OK)
        {
            b_waiting = true;
            started   = now;
            LOG_INFO(TAG_S2_BENCH, "turn %u starting", stage + 1u);
        }
        else if ((uint32_t)(now - last_warn) >= 2000u)
        {
            last_warn = now;
            LOG_WARN(TAG_S2_BENCH, "turn %u rejected: result=%d state=%d",
                     stage + 1u, (int)res, (int)p_ctl->status.state);
        }
        else
        {
            /* No action needed in any other case. */
        }
    }
    else
    {
        /* No action needed in any other case. */
    }
}
#else

/* Standalone closed-loop sequence uses the same motor task and encoders. */
void robot_control_step(MOTION_CONTROLLER * p_ctl, uint32_t now,
                        int32_t left_ticks, int32_t right_ticks)
{
    static const MOTION_COMMAND sequence[] = {
        {MOTION_CMD_FORWARD_CM, 20.0f},
        {MOTION_CMD_TURN_LEFT_DEG, 90.0f},
        {MOTION_CMD_FORWARD_CM, 20.0f},
        {MOTION_CMD_TURN_RIGHT_DEG, 90.0f}};
    static unsigned stage;
    static bool     b_waiting, b_done;
    static uint32_t started, pause_until = S2_ROBOT_START_MS;
    MOTION_COMMAND  stop = {MOTION_CMD_STOP, 0.0f};
    if (b_done)
    {
        (void)motion_control_command(p_ctl, &stop, now, left_ticks,
                                     right_ticks);
        return;
    }
    if (p_ctl->status.state == MOTION_STATE_FAULT)
    {
        return;
    }
    if (b_waiting)
    {
        if (p_ctl->status.state == MOTION_STATE_IDLE &&
            p_ctl->status.motion_complete)
        {
            b_waiting   = false;
            pause_until = now + 1500u;
            if (++stage == sizeof(sequence) / sizeof(sequence[0]))
            {
                b_done = true;
                LOG_INFO(TAG_S2_DEMO, "finished; motors OFF");
            }
        }
        else if ((uint32_t)(now - started) > S2_DEMO_TIMEOUT_MS)
        {
            (void)motion_control_command(p_ctl, &stop, now, left_ticks,
                                         right_ticks);
            b_done = true;
            LOG_WARN(TAG_S2_DEMO, "timeout; motors OFF");
        }
        else
        {
            /* No action needed in any other case. */
        }
    }
    else if ((int32_t)(now - pause_until) >= 0)
    {
        if (motion_control_command(p_ctl, &sequence[stage], now, left_ticks,
                                   right_ticks) == MOTION_RESULT_OK)
        {
            b_waiting = true;
            started   = now;
            LOG_INFO(TAG_S2_DEMO, "stage %u started", stage + 1u);
        }
    }
    else
    {
        /* No action needed in any other case. */
    }
}
#endif /* S2_PID_BENCH / S2_PIVOT_BENCH */
#else
void robot_control_step(MOTION_CONTROLLER * p_ctl, uint32_t now,
                        int32_t left_ticks, int32_t right_ticks)
{
    (void)p_ctl;
    (void)now;
    (void)left_ticks;
    (void)right_ticks;
}
#endif
