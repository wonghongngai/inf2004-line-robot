/* Adapter around the existing S3/S4 public interfaces. */
#include "subsystem2.h"
#include "robot_control.h"

#if APP_ROBOT_INTEGRATED && !APP_TEST
#include "../subsystem3/line_service.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem5/s5_executor.h"
#include "../subsystem1/subsystem1.h"

static bool armed, halted, line_halt, turning, configured;
/* #8 straddle line model (S2_LINE_STRADDLE); runtime so host tests can
   exercise both models. */
static bool straddle = S2_LINE_STRADDLE;
static unsigned steer_action;            /* 0 straight, 1 right, 2 left */
static unsigned steer_candidate;
static uint32_t candidate_since, action_since, clear_since, junction_done;
static bool clear_pending, leaving_junction;
static uint8_t pending_letter;           /* bit 7 = waiting, bits 4..0 = A..Z */
#define TURN_ADVANCE 1u
#define TURN_PIVOT 2u
static unsigned turn_stage, turn_cmd;
static int32_t uturn_r0;                 /* right ticks when the U-turn began */
static int32_t pivot_l0, pivot_r0;       /* wheel ticks when a left/right pivot began */
/* #48 Ready state: arm only once START has arrived (or auto_start is set).
   A START that arrives while still initialising is kept until Ready. */
static bool auto_start = S2_AUTO_START, start_requested, ready_reported;

static uint32_t last_imu_sample, imu_count, imu_errors, resumed_at, turn_started;
static uint32_t rearm_since;
static int32_t rearm_l, rearm_r;
/* S5 obstacle-avoidance hook state (see avoid_step below). */
static S5_EXEC vc_s5;

/* Line stops (lost, line fault, bad snapshot) may re-arm by
   themselves; any other stop is a safety stop and latches until reboot. */
static void stop_robot(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l,
                       int32_t r, const char *reason, bool recoverable)
{
    MOTION_COMMAND stop = {MOTION_CMD_STOP, 0.0f};
    (void)motion_control_command(ctl, &stop, now, l, r);
    if (!halted) {
        /* Tell S5 the robot is no longer following (no new obstacles). */
        s5_exec_set_following(&vc_s5, FALSE);
        (void)line_service_enable(false);
        LOG_WARN(TAG_S2_ROBOT, "stopped: %s; %s", reason,
                  recoverable ? "re-arms when back on the line"
                              : "reboot to re-arm");
        line_halt = recoverable;
        rearm_since = now;
        rearm_l = l;
        rearm_r = r;
    } else if (!recoverable) {
        line_halt = false;
    }
    halted = true;
}

static void halt(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r,
                 const char *reason)
{
    stop_robot(ctl, now, l, r, reason, false);
}

static void halt_line(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l,
                      int32_t r, const char *reason)
{
    stop_robot(ctl, now, l, r, reason, true);
}

/* Paused S3 snapshots still carry calibrated sensor bits. Re-arm only after
   centre and right have both seen black, with the wheels still, for
   S2_LINE_REARM_MS; moving the robot by hand restarts the wait. */
static bool line_rearm_ready(uint32_t now, int32_t l, int32_t r)
{
    line_barcode_msg_t msg = {{0u, 0u, 0u}};
    ER er = line_service_snapshot(&msg);
    /* Straddle (#8): both-white is the normal on-line reading, so any healthy
       snapshot will do; the S3 model needs centre + right on black. */
    bool on_line = straddle ? (msg.data[1] & 7u) != LINE_BARCODE_FAULT :
                              (msg.data[0] & 3u) == 3u;
    bool ready = er >= E_OK && on_line && l == rearm_l && r == rearm_r;

    if (!ready) {
        rearm_since = now;
        rearm_l = l;
        rearm_r = r;
        return false;
    }
    return (uint32_t)(now - rearm_since) >= S2_LINE_REARM_MS;
}

static bool line_unsafe_active;
static uint32_t line_unsafe_since;

/* A FAULT stops at once. Stale, paused, searching or absent-position
   snapshots must persist for S2_LINE_UNSAFE_MS before stopping, so one late
   or debouncing sample cannot latch a stop. S3 queues LOST after 100 ms. */
static bool line_snapshot_ok(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l,
                             int32_t r, ER er, const line_barcode_msg_t *msg,
                             bool need_position)
{
    unsigned state = msg->data[1] & 7u;
    unsigned pos = (msg->data[0] >> 3u) & 7u;
    bool fault = er >= E_OK && state == LINE_BARCODE_FAULT;

    if (er >= E_OK && !fault && state != LINE_BARCODE_SEARCH &&
        state != LINE_BARCODE_PAUSED && (!need_position || pos < 6u)) {
        line_unsafe_active = false;
        return true;
    }
    if (!line_unsafe_active) {
        line_unsafe_active = true;
        line_unsafe_since = now;
    }
    if (fault || (uint32_t)(now - line_unsafe_since) >= S2_LINE_UNSAFE_MS) {
        LOG_WARN(TAG_S2_ROBOT, "line er=%d state=%u pos=%u sensors=%u",
                  (int)er, state, pos, (unsigned)(msg->data[0] & 7u));
        halt_line(ctl, now, l, r, "line snapshot unsafe or stale");
    }
    return false;
}

/* ---- S5 obstacle-avoidance hook (added by Buddy 5; IF-10/IF-11) --------
   The VC stays the only motor commander. S5 only asks, through its request
   word; the shared executor (subsystem5/s5_executor.c) runs the hand-shake
   and calls back into the functions below, which drive through the motion
   controller. */
typedef struct {
    MOTION_CONTROLLER *ctl;
    uint32_t now;
    int32_t l, r;
} VC_CTX;

static void vc_stop(void *p)
{
    VC_CTX *c = p;
    MOTION_COMMAND stop = {MOTION_CMD_STOP, 0.0f};
    (void)motion_control_command(c->ctl, &stop, c->now, c->l, c->r);
}

static BOOL vc_start_move(void *p, const S5_REQUEST *req)
{
    VC_CTX *c = p;
    MOTION_COMMAND cmd;
    motion_control_step(c->ctl, c->now, c->l, c->r); /* STOPPING -> IDLE */
    cmd.type = req->move == S5_MOVE_FWD ? MOTION_CMD_FORWARD_CM :
               req->move == S5_MOVE_TURN_L ? MOTION_CMD_TURN_LEFT_DEG :
               MOTION_CMD_TURN_RIGHT_DEG;
    cmd.value = (float)req->value;
    /* all_test's S5 moves: heading-held 12.5/15 % legs, 20 % pivots. */
    motion_control_set_profile(c->ctl, MOTION_PROFILE_AVOID);

    return motion_control_command(c->ctl, &cmd, c->now, c->l, c->r) == MOTION_RESULT_OK;
}

static UB vc_run_move(void *p, const S5_REQUEST *req)
{
    VC_CTX *c = p;
    (void)req;
    /* The motion controller runs forward moves and pivots itself. */
    return (c->ctl->status.state == MOTION_STATE_IDLE && c->ctl->status.motion_complete)
               ? S5_EXEC_DONE : S5_EXEC_RUNNING;
}

static BOOL vc_is_stopped(void *p)
{
    VC_CTX *c = p;
    return c->ctl->status.state == MOTION_STATE_IDLE;
}

/* Steering sensors (SIDE layout: C and R; L is the barcode scanner). */
#define VC_LINE_MASK (S2_LINE_LAYOUT == LINE_BARCODE_LAYOUT_SIDE ? \
                      3u : LINE_BARCODE_SENSOR_MASK)

static UB vc_line_sensors(void *p)
{
    line_barcode_msg_t msg = {{0u, 0u, 0u}};
    (void)p;
    return line_service_snapshot(&msg) >= E_OK ?
           (UB)(msg.data[0] & VC_LINE_MASK) : 0u;
}

static BOOL vc_take_control(void *p)
{
    VC_CTX *c = p;
    turning = false; /* obstacle wins over a barcode turn */
    turn_stage = 0u;
    if (line_service_enable(false) < E_OK) {
        halt(c->ctl, c->now, c->l, c->r, "line pause failed");
        return false;
    }
    LOG_INFO(TAG_S2_ROBOT, "obstacle: S5 has control");
    return true;
}

/* Same steps as the end of a barcode turn. */
static BOOL vc_give_back(void *p)
{
    VC_CTX *c = p;
    if (line_service_enable(true) < E_OK) {
        halt(c->ctl, c->now, c->l, c->r, "line resume failed");
        return false;
    }
    resumed_at = c->now;
    line_unsafe_active = false;
    /* Rejoining at an angle can put C and R both on the line for a moment;
       as all_test, treat it like the crossing just left by a turn. */
    leaving_junction = straddle;
    junction_done = c->now;
    steer_action = steer_candidate = 0u;
    clear_pending = false;
    LOG_INFO(TAG_S2_ROBOT, "obstacle handled, line following");
    return true;
}

static void vc_safe_stop(void *p, const char *reason)
{
    VC_CTX *c = p;
    halt(c->ctl, c->now, c->l, c->r, reason);
}

static BOOL vc_may_follow(void *p)
{
    (void)p;
    return !turning;
}

static const S5_EXEC_OPS vc_s5_ops = {
    vc_stop, vc_start_move, vc_run_move, vc_is_stopped,
    vc_line_sensors, vc_take_control, vc_give_back, vc_safe_stop, vc_may_follow,
    S2_AVOID_HOLD_SETTLE_MS,  /* all_test: let the robot coast to a stop */
    S2_AVOID_MOVE_TIMEOUT_MS, /* all_test: 20 s per leg or pivot */
    VC_LINE_MASK
};

/* Returns true while S5 has control, so line following is skipped. */
static bool avoid_step(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    VC_CTX c = {ctl, now, l, r};
    return s5_exec_step(&vc_s5, &vc_s5_ops, &c, now);
}

/* #72 MQTT link. PAUSE stops at once and blocks line re-arm until RESUME;
   RESUME restarts only on a healthy line and IMU; safety stops still need a
   reboot. The letter map arrives over MQTT; until then junctions go straight. */
static bool paused, map_received, status_requested;
static unsigned char map_letters[S1_MAP_SLOTS];   /* 0..25 = A..Z */
static unsigned last_letter;                      /* 0 none, 1..26 = A..Z */
static IMU_DATA last_imu;
static bool have_imu, in_avoid;
static uint32_t last_telemetry;
static ER telemetry_error;
static int32_t odo_l, odo_r;
static bool odo_started;
static float distance_cm;

static void vc_stop_motors(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    MOTION_COMMAND stop = {MOTION_CMD_STOP, 0.0f};
    (void)motion_control_command(ctl, &stop, now, l, r);
}

/* The same healthy-IMU test that keeps the robot running. */
static bool imu_healthy(uint32_t now)
{
    return have_imu && last_imu.sensor_ok && last_imu.calibrated &&
           (!armed || last_imu.read_errors == imu_errors) &&
           (uint32_t)(now - last_imu_sample) <= S2_IMU_STALE_MS &&
           last_imu.motion_state != IMU_MOTION_LIFTED &&
           last_imu.motion_state != IMU_MOTION_FAULT;
}

static void pause_robot(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    if (!armed) {
        start_requested = false; /* a PAUSE in Ready cancels a pending START */
        LOG_INFO(TAG_S2_ROBOT, "PAUSE in Ready: pending START cancelled");
        return;
    }
    if (halted && !line_halt) {
        LOG_WARN(TAG_S2_ROBOT, "PAUSE ignored: already in a safety stop");
        return;
    }
    vc_stop_motors(ctl, now, l, r);
    if (!paused) {
        s5_exec_set_following(&vc_s5, FALSE);
        (void)line_service_enable(false);
        turning = false;
        turn_stage = 0u;
        paused = true;
        LOG_INFO(TAG_S2_ROBOT, "paused (MQTT); RESUME to continue");
    }
}

static void resume_robot(uint32_t now)
{
    line_barcode_msg_t msg = {{0u, 0u, 0u}};
    if (!paused) {
        LOG_WARN(TAG_S2_ROBOT, "RESUME ignored: not paused%s",
                  armed ? "" : " (use START to leave Ready)");
        return;
    }
    if (!imu_healthy(now)) {
        LOG_WARN(TAG_S2_ROBOT, "RESUME refused: IMU not healthy; still paused");
        return;
    }
    /* Same line test as the automatic re-arm. */
    if (line_service_snapshot(&msg) < E_OK ||
        (straddle ? (msg.data[1] & 7u) == LINE_BARCODE_FAULT : (msg.data[0] & 3u) != 3u)) {
        LOG_WARN(TAG_S2_ROBOT, "RESUME refused: put the robot on the line; still paused");
        return;
    }
    paused = false;
    if (halted) {
        /* The recoverable line-stop path re-arms as usual, but the "held
           still" wait starts now: time spent paused does not count. */
        rearm_since = now;
        LOG_INFO(TAG_S2_ROBOT, "resumed; re-arms when the line is held still");
        return;
    }
    if (line_service_enable(true) < E_OK) {
        paused = true;
        LOG_ERR(TAG_S2_ROBOT, "RESUME failed: line service; still paused");
        return;
    }
    resumed_at = now;
    line_unsafe_active = false;
    LOG_INFO(TAG_S2_ROBOT, "resumed (MQTT)");
}

/* Drain up to one queue's worth of MQTT commands without blocking. */
static void poll_commands(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    for (unsigned i = 0u; i < S1_QUEUE_DEPTH; ++i) {
        S1_COMMAND command;
        if (subsystem1_receive_command(&command, TMO_POL) < E_OK) return;
        switch (command.bytes[0]) {
        case S1_COMMAND_STATUS:
            status_requested = true;
            break;
        case S1_COMMAND_PAUSE:
            pause_robot(ctl, now, l, r);
            break;
        case S1_COMMAND_RESUME:
            resume_robot(now);
            break;
        case S1_COMMAND_START:
            if (armed) {
                LOG_WARN(TAG_S2_ROBOT, "START ignored: already started%s",
                          halted && !line_halt ? " (safety stop: reboot to retry)" :
                          paused ? " (paused: use RESUME)" : "");
            } else if (!start_requested) {
                start_requested = true;
                LOG_INFO(TAG_S2_ROBOT, "START received%s",
                          ready_reported ? "" : "; starts once initialisation finishes");
            }
            break;
        default: /* RESET_STATS, REVERSE: defined, not supported here */
            LOG_WARN(TAG_S2_ROBOT, "command %u not supported by the integrated VC",
                      (unsigned)command.bytes[0]);
            break;
        }
    }
}

/* Apply the newest valid letter map from MQTT (S1 already validated it). */
static void poll_mapping(void)
{
    for (unsigned i = 0u; i < S1_QUEUE_DEPTH; ++i) {
        S1_MAPPING mapping;
        unsigned char letters[S1_MAP_SLOTS];
        if (subsystem1_receive_mapping(&mapping, TMO_POL) < E_OK) return;
        if (!s1_mapping_get(&mapping, letters)) continue;
        if (line_service_set_mapping((char)('A' + letters[S1_MAP_LEFT]),
                                     (char)('A' + letters[S1_MAP_RIGHT]),
                                     (char)('A' + letters[S1_MAP_STRAIGHT]),
                                     (char)('A' + letters[S1_MAP_UTURN])) < E_OK) {
            LOG_WARN(TAG_S2_ROBOT, "MQTT map rejected by the line service");
            continue;
        }
        for (unsigned s = 0u; s < S1_MAP_SLOTS; ++s) map_letters[s] = letters[s];
        map_received = true;
        LOG_INFO(TAG_S2_ROBOT, "MQTT map applied: %c=left %c=right %c=straight %c=U-turn",
                  'A' + letters[0], 'A' + letters[1], 'A' + letters[2], 'A' + letters[3]);
    }
}

/* Telemetry direction for the last letter under the current map. */
static unsigned letter_direction(void)
{
    if (!map_received || last_letter == 0u) return S1_BARCODE_NONE;
    for (unsigned s = 0u; s < S1_MAP_SLOTS; ++s) {
        if (map_letters[s] == last_letter - 1u) return S1_BARCODE_LEFT + s;
    }
    return S1_BARCODE_NONE;
}

static void update_distance(const MOTION_CONTROLLER *ctl, int32_t l, int32_t r)
{
    if (odo_started && ctl->cfg.left_ticks_per_cm > 0.0f &&
        ctl->cfg.right_ticks_per_cm > 0.0f) {
        int32_t dl = l - odo_l, dr = r - odo_r;
        if (dl < 0) dl = -dl;
        if (dr < 0) dr = -dr;
        distance_cm += 0.5f * ((float)dl / ctl->cfg.left_ticks_per_cm +
                               (float)dr / ctl->cfg.right_ticks_per_cm);
    }
    odo_l = l;
    odo_r = r;
    odo_started = true;
}

static unsigned robot_state(void)
{
    if (halted) return line_halt ? S1_STATE_RECOVERING : S1_STATE_FAULT;
    if (paused || !armed) return S1_STATE_IDLE;
    if (in_avoid) return S1_STATE_AVOIDING;
    return turning ? S1_STATE_TURNING : S1_STATE_FOLLOWING;
}

/* all_test's barcode tuning log: each new S3 frame's element widths, at
   most every 500 ms, so a misread can be diagnosed from the serial log. */
static uint32_t frame_report_ms, frame_sequence;

static void report_barcode_frame(uint32_t now)
{
    line_service_barcode_frame_t frame = {0};

    if ((uint32_t)(now - frame_report_ms) < 500u) return;
    frame_report_ms = now;
    line_service_barcode_frame(&frame);
    if (frame.sequence == frame_sequence) return;
    LOG_INFO(TAG_S2_BARCODE, "frame id=%lu runs=%u/29 black=%u white=%u result=%c skipped=%lu",
              (unsigned long)frame.sequence, (unsigned)frame.count,
              (unsigned)((frame.count + 1u) / 2u), (unsigned)(frame.count / 2u),
              frame.letter < 26u ? 'A' + frame.letter : '?',
              (unsigned long)(frame.sequence - frame_sequence - 1u));
    for (unsigned i = 0; i < frame.count; ++i)
        LOG_INFO(TAG_S2_BARCODE, "width frame=%lu run=%u %s width_ms=%u",
                  (unsigned long)frame.sequence, i + 1u,
                  (i & 1u) ? "WHITE" : "BLACK", (unsigned)frame.widths[i]);
    frame_sequence = frame.sequence;
}

/* IF-05 telemetry, once a second (or at once after STATUS). Nonblocking. */
static void report_telemetry(const MOTION_CONTROLLER *ctl, uint32_t now)
{
    S1_TELEMETRY packet;
    line_barcode_msg_t msg = {{0u, 0u, 0u}};
    unsigned faults = 0u, black = 0u, white = 0u, hump, sensors = 0u;
    uint8_t debug_letter;
    S5_OBSTACLE_INFO obstacle;
    float speed;
    ER er;

    if (!status_requested && (uint32_t)(now - last_telemetry) < 1000u) return;
    status_requested = false;
    last_telemetry = now;
    if (!imu_healthy(now)) faults |= S1_FAULT_IMU;
    if (ctl->status.fault == MOTION_FAULT_LEFT_ENCODER_STALL ||
        ctl->status.fault == MOTION_FAULT_RIGHT_ENCODER_STALL) faults |= S1_FAULT_MOTOR_STALL;
    if (line_service_snapshot(&msg) >= E_OK) {
        unsigned s3 = msg.data[0] & 7u; /* S3: L/C/R bits 2/1/0; wire: 0/1/2 */
        sensors = ((s3 & 4u) >> 2u) | (s3 & 2u) | ((s3 & 1u) << 2u);
    }
    line_service_barcode_debug(&black, &white, &debug_letter);
    hump = have_imu ? last_imu.ramp_height_mm : 0u;
    speed = 0.5f * (ctl->status.left_speed_cm_s + ctl->status.right_speed_cm_s);
    if (speed > (float)S1_SPEED_MAX) speed = (float)S1_SPEED_MAX;
    if (speed < (float)S1_SPEED_MIN) speed = (float)S1_SPEED_MIN;

    s1_telemetry_init(&packet);
    (void)s1_telemetry_set(&packet, S1_FIELD_STATE, (int)robot_state());
    (void)s1_telemetry_set(&packet, S1_FIELD_FAULTS, (int)faults);
    (void)s1_telemetry_set(&packet, S1_FIELD_SPEED_CMS, (int)speed);
    (void)s1_telemetry_set(&packet, S1_FIELD_SPEED_VALID, 1);
    (void)s1_telemetry_set(&packet, S1_FIELD_DISTANCE_CM,
                           distance_cm >= (float)S1_DISTANCE_MAX ? S1_DISTANCE_MAX : (int)distance_cm);
    (void)s1_telemetry_set(&packet, S1_FIELD_LINE_SENSORS, (int)sensors);
    (void)s1_telemetry_set(&packet, S1_FIELD_BARCODE, (int)letter_direction());
    (void)s1_telemetry_set(&packet, S1_FIELD_BARCODE_LETTER, (int)last_letter);
    (void)s1_telemetry_set(&packet, S1_FIELD_MAP_RECEIVED, map_received ? 1 : 0);
    (void)s1_telemetry_set(&packet, S1_FIELD_BARCODE_BLACK,
                           black > S1_BARCODE_COUNT_MAX ? (int)S1_BARCODE_COUNT_MAX : (int)black);
    (void)s1_telemetry_set(&packet, S1_FIELD_BARCODE_WHITE,
                           white > S1_BARCODE_COUNT_MAX ? (int)S1_BARCODE_COUNT_MAX : (int)white);
    (void)s1_telemetry_set(&packet, S1_FIELD_HIGHEST_HUMP_MM, hump > S1_HUMP_MAX ? S1_HUMP_MAX : (int)hump);
    /* #40: S5's last obstacle scan (all 0 until one found an obstacle; a
       corrupt word reads as 0 too). S5's ranges equal S1's. */
    if (subsystem5_get_obstacle(&obstacle) == E_OK) {
        (void)s1_telemetry_set(&packet, S1_FIELD_OBSTACLE_CM, (int)obstacle.distance_cm);
        (void)s1_telemetry_set(&packet, S1_FIELD_SCAN_DEG, (int)obstacle.angle_deg);
        (void)s1_telemetry_set(&packet, S1_FIELD_OBSTACLE_WIDTH_CM, (int)obstacle.width_cm);
    }
    /* Battery stays a 0 placeholder: nothing measures it. */
    er = subsystem1_publish_telemetry(&packet);
    if (er != telemetry_error && er != E_NOSPT) {
        LOG_INFO(TAG_S2_ROBOT, "telemetry queue %s (%d)",
                  er >= E_OK ? "accepting packets again" :
                  er == E_TMOUT ? "full; waiting for MQTT" : "error", (int)er);
    }
    telemetry_error = er;
}

/* ---- #8 straddle line model, ported from all_test's white_follow_step ----
   C and R sit either side of the line. S3's SEARCH/LOST/JUNCTION/COMMAND
   events assume the line under both sensors, so they are ignored here; S3
   faults still stop the robot, and S3 still decodes barcodes. */

/* Junction (C/R both black): a mapped stored letter drives onto the crossing,
   then pivots; otherwise go straight (decided 21 Nov). Returns true when a
   turn sequence started. */
static bool straddle_junction(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    MOTION_COMMAND cmd;
    uint8_t command;
    unsigned letter = pending_letter & LINE_BARCODE_LETTER_MASK;
    bool waiting = (pending_letter & 0x80u) != 0u;

    pending_letter = 0u;
    leaving_junction = true;
    junction_done = now;
    steer_action = 0u;
    clear_pending = false;
    if (!waiting || line_service_lookup((uint8_t)letter, &command) != E_OK) {
        LOG_INFO(TAG_S2_ROBOT, "junction: %s, straight on",
                  !waiting ? "no barcode" : map_received ? "no mapped barcode" :
                  "no MQTT letter map yet");
        return false;
    }
    if (line_service_enable(false) < E_OK) {
        halt(ctl, now, l, r, "line pause failed");
        return true;
    }
    /* all_test drives straight on from line following (JUNCTION profile);
       without profiles, stop first as the base moves need IDLE. */
    motion_control_set_profile(ctl, MOTION_PROFILE_JUNCTION);
    if (ctl->cfg.move_left_pwm_percent <= 0.0f) {
        vc_stop_motors(ctl, now, l, r);
        motion_control_step(ctl, now, l, r); /* STOPPING -> IDLE */
    }
    cmd = (MOTION_COMMAND){MOTION_CMD_FORWARD_CM,
                           (command & 3u) == LINE_BARCODE_CMD_RIGHT ?
                           S2_TURN_ADVANCE_RIGHT_CM : S2_TURN_ADVANCE_LEFT_CM};
    if (motion_control_command(ctl, &cmd, now, l, r) != MOTION_RESULT_OK) {
        halt(ctl, now, l, r, "junction advance rejected");
        return true;
    }
    turning = true;
    turn_started = now;
    turn_stage = TURN_ADVANCE;
    turn_cmd = command & 3u;
    LOG_INFO(TAG_S2_ROBOT, "junction: letter '%c' -> %s", 'A' + letter,
              turn_cmd == LINE_BARCODE_CMD_LEFT ? "LEFT" :
              turn_cmd == LINE_BARCODE_CMD_RIGHT ? "RIGHT" :
              turn_cmd == LINE_BARCODE_CMD_STRAIGHT ? "STRAIGHT" : "U-TURN");
    return true;
}

static void straddle_step(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    line_barcode_msg_t msg = {{0u, 0u, 0u}};
    MOTION_COMMAND cmd;
    ER er = line_service_snapshot(&msg);
    unsigned state = msg.data[1] & 7u, sensors;

    /* S3 faults stop at once; a missing or stuck-paused snapshot only after
       S2_LINE_UNSAFE_MS, as in the S3 model. */
    if (er >= E_OK && state == LINE_BARCODE_FAULT) {
        halt_line(ctl, now, l, r, "line sensor fault");
        return;
    }
    if (er < E_OK || state == LINE_BARCODE_PAUSED) {
        if (!line_unsafe_active) {
            line_unsafe_active = true;
            line_unsafe_since = now;
        }
        if ((uint32_t)(now - line_unsafe_since) >= S2_LINE_UNSAFE_MS)
            halt_line(ctl, now, l, r, "line snapshot unsafe or stale");
        return;
    }
    line_unsafe_active = false;
    /* all_test slows down while GP28 is on a bar or a capture is open. */
    motion_control_set_scanning(ctl, S2_LINE_LAYOUT == LINE_BARCODE_LAYOUT_SIDE &&
                                ((msg.data[0] & LINE_BARCODE_LEFT) != 0u ||
                                 (msg.data[1] & LINE_BARCODE_CAPTURE_ACTIVE) != 0u));

    /* Barcodes: remember the newest letter until the next junction. */
    for (unsigned i = 0; i < 24u; ++i) {
        line_barcode_msg_t ev;
        er = line_service_receive(&ev, TMO_POL);
        if (er == E_TMOUT) break;
        if (er < E_OK) {
            halt(ctl, now, l, r, "line receive failed");
            return;
        }
        unsigned event = (ev.data[1] >> 3u) & 7u;
        if (event == LINE_BARCODE_EVT_FAULT) {
            halt_line(ctl, now, l, r, "line sensor fault");
            return;
        }
        if (event == LINE_BARCODE_EVT_INVALID)
            LOG_WARN(TAG_S2_BARCODE, "rejected invalid frame: check start/stop, narrow/wide widths and white margins");
        if (event == LINE_BARCODE_EVT_BARCODE) {
            unsigned letter = ev.data[2] & LINE_BARCODE_LETTER_MASK;
            uint8_t command;
            pending_letter = (uint8_t)(0x80u | letter);
            last_letter = letter + 1u;
            LOG_INFO(TAG_S2_BARCODE, "decoded=%c queued=%s; execute at next C/R junction",
                      'A' + letter,
                      line_service_lookup((uint8_t)letter, &command) != E_OK ? "no mapping" :
                      command == LINE_BARCODE_CMD_LEFT ? "LEFT" :
                      command == LINE_BARCODE_CMD_RIGHT ? "RIGHT" :
                      command == LINE_BARCODE_CMD_STRAIGHT ? "STRAIGHT" : "U-TURN");
        }
    }

    sensors = msg.data[0] & 3u; /* C bit 1, R bit 0; the scanner is ignored */
    if (leaving_junction && sensors != 3u &&
        (uint32_t)(now - junction_done) >= S2_JUNCTION_SETTLE_MS)
        leaving_junction = false;
    if (sensors == 3u && !leaving_junction && straddle_junction(ctl, now, l, r)) return;

    if (sensors == 1u || sensors == 2u) {
        if (sensors != steer_candidate) {
            steer_candidate = sensors;
            candidate_since = now;
        }
        /* A first correction acts at once; only a reversal must persist. */
        if (steer_action == 0u ||
            (uint32_t)(now - candidate_since) >= S2_LINE_CONFIRM_MS) {
            if (steer_action != sensors) action_since = now;
            steer_action = sensors;
        }
        clear_pending = false;
    } else if (sensors == 0u && steer_action != 0u) {
        steer_candidate = 0u;
        if (!clear_pending) {
            clear_pending = true;
            clear_since = now;
        }
        if ((uint32_t)(now - clear_since) >= S2_WHITE_CLEAR_MS) {
            steer_action = 0u;
            clear_pending = false;
        }
    }
    if (sensors == 0u) steer_candidate = 0u;

    /* S2-U09 diagnosis (22 Nov): once a second, how many control steps each
       sensor pattern and each steering action lasted. sensors: 0 neither
       black, 1 R only (drifting left, steer right), 2 C only (drifting
       right, steer left), 3 both; act: 0 straight, 1 right, 2 left. */
    {
        static uint32_t log_t0, log_n;
        static uint16_t sens_n[4], act_n[3];
        if (log_n == 0u) log_t0 = now;
        log_n++;
        sens_n[sensors & 3u]++;
        act_n[steer_action > 2u ? 0u : steer_action]++;
        if ((uint32_t)(now - log_t0) >= 1000u) {
            LOG_INFO(TAG_S2_ROBOT, "steer n=%u sens none=%u R=%u C=%u both=%u act straight=%u right=%u left=%u",
                      (unsigned)log_n, sens_n[0], sens_n[1], sens_n[2], sens_n[3],
                      act_n[0], act_n[1], act_n[2]);
            log_n = 0u;
            for (unsigned i = 0; i < 4u; ++i) sens_n[i] = 0u;
            for (unsigned i = 0; i < 3u; ++i) act_n[i] = 0u;
        }
    }

    /* Right sensor black: drifting left, steer right (positive). */
    float strength = (uint32_t)(now - action_since) >= S2_LINE_HARD_MS ? 1.0f : 0.5f;
    cmd.type = MOTION_CMD_FOLLOW_LINE;
    cmd.value = sensors == 3u ? 0.0f :         /* on the crossing: straight */
                steer_action == 1u ? strength :
                steer_action == 2u ? -strength : 0.0f;
    if (motion_control_command(ctl, &cmd, now, l, r) != MOTION_RESULT_OK)
        halt(ctl, now, l, r, "follow command rejected");
}

static void control_step(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    IMU_DATA imu;
    line_barcode_msg_t msg = {{0u, 0u, 0u}};
    MOTION_COMMAND cmd;
    ER er;

    in_avoid = false;
    if (halted) {
        /* PAUSE also blocks the automatic line re-arm. */
        if (!line_halt || paused || !line_rearm_ready(now, l, r)) {
            cmd = (MOTION_COMMAND){MOTION_CMD_STOP, 0.0f};
            (void)motion_control_command(ctl, &cmd, now, l, r);
            return;
        }
        halted = false;
        line_halt = false;
        turning = false;
        er = line_service_enable(true);
        if (er < E_OK) {
            halt(ctl, now, l, r, "line resume failed");
            return;
        }
        resumed_at = now;
        line_unsafe_active = false;
        LOG_INFO(TAG_S2_ROBOT, "line found, re-armed");
        return;
    }
    if (paused) {
        vc_stop_motors(ctl, now, l, r);
        return;
    }
    if (ctl->status.state == MOTION_STATE_FAULT) {
        halt(ctl, now, l, r, "motion fault / calibration missing");
        return;
    }
    /* robot_control_step() reads S4 once per step into last_imu. */
    if (!have_imu) {
        halt(ctl, now, l, r, "IMU snapshot failed");
        return;
    }
    imu = last_imu;
    if (!armed) {
        /* Remain stationary throughout IMU baseline calibration, and while
           S5 holds at start-up (its servo swing shakes the IMU). */
        if (s5_exec_startup_hold()) return;
        if (!S2_LINE_CALIBRATION_READY || now < S2_ROBOT_START_MS ||
            !imu.sensor_ok || !imu.calibrated || imu_count == 0 ||
            (uint32_t)(now - last_imu_sample) > S2_IMU_STALE_MS) return;
        /* Ready (#48): initialised, motors off, waiting for START. */
        if (!auto_start && !start_requested) {
            if (!ready_reported) {
                ready_reported = true;
                LOG_INFO(TAG_S2_ROBOT, "ready: motors off, waiting for START over MQTT");
            }
            return;
        }
        if (!configured) {
            er = line_service_calibrate(S2_LINE_BLACK_RAW, S2_LINE_WHITE_RAW);
            if (er == E_OK) er = line_service_set_layout(S2_LINE_LAYOUT);
            /* No built-in letter map (#72): the map arrives over MQTT. */
            if (er == E_OK) er = line_service_enable(true);
            if (er < E_OK) {
                halt(ctl, now, l, r, "line configuration failed");
                return;
            }
            configured = true;
            resumed_at = now;
            imu_errors = imu.read_errors;
            armed = true;
            LOG_INFO(TAG_S2_ROBOT, "armed: %s", map_received ?
                      "letter map from MQTT" : "no letter map yet, straight at junctions");
        }
    }
    /* Tilt is report-only (#67, decided 20 Nov 2026): S4 still logs it and
       S5 ignores ultrasonic readings while tilted, but it never stops the
       robot. A false tilt would otherwise latch a stop until reboot.
       Lifting the robot (#53, replaces collision) stops it and the stop
       latches until reboot, like every IMU stop. */
    bool lifted = imu.motion_state == IMU_MOTION_LIFTED ||
                  imu.event == IMU_EVENT_LIFT;
    if (lifted || !imu.sensor_ok || !imu.calibrated ||
        imu.read_errors != imu_errors ||
        (uint32_t)(now - last_imu_sample) > S2_IMU_STALE_MS ||
        imu.motion_state == IMU_MOTION_FAULT ||
        imu.event == IMU_EVENT_FAULT) {
        halt(ctl, now, l, r, lifted ? "robot lifted (IMU)" : "IMU unsafe or stale");
        /* all_test: the values that latched the stop, printed once. */
        LOG_INFO(TAG_S2_ROBOT, "IMU stop data: age=%lu ms samples=%lu ok=%d calibrated=%d motion=%d event=%d errors=%lu baseline_errors=%lu",
                  (unsigned long)(now - last_imu_sample), (unsigned long)imu_count,
                  (int)imu.sensor_ok, (int)imu.calibrated, (int)imu.motion_state,
                  (int)imu.event, (unsigned long)imu.read_errors,
                  (unsigned long)imu_errors);
        return;
    }
    /* S5 obstacle avoidance takes control while it has a request. */
    in_avoid = avoid_step(ctl, now, l, r);
    if (in_avoid) return;
    /* Hump is terrain information; keep following, without inventing speed
       compensation before physical tuning. */
    if (turning) {
        bool uturn = ctl->cfg.move_left_pwm_percent > 0.0f &&
                     turn_stage == TURN_PIVOT && turn_cmd == LINE_BARCODE_CMD_UTURN;

        if (uturn) {
            /* all_test: the counter-rotating U-turn ends once the right wheel
               has covered half its 180-degree pivot arc AND the centre
               sensor is back on black. */
            int32_t moved = r - uturn_r0;
            float target = 180.0f * ctl->cfg.right_turn_ticks_per_degree * 0.5f;
            if (moved < 0) moved = -moved;
            if ((float)moved >= target && line_service_snapshot(&msg) >= E_OK &&
                (msg.data[0] & LINE_BARCODE_CENTRE) != 0u) {
                vc_stop_motors(ctl, now, l, r);
                motion_control_step(ctl, now, l, r); /* STOPPING -> IDLE */
            }
        }
        /* 22 Nov: a left/right pivot that is past S2_PIVOT_LINE_MIN_FRACTION of
           its arc stops as soon as the centre sensor sees the line, instead of
           turning on past it to the full tick count. Without a line it still
           ends on the full count. */
        if (ctl->cfg.move_left_pwm_percent > 0.0f && turn_stage == TURN_PIVOT &&
            (turn_cmd == LINE_BARCODE_CMD_LEFT || turn_cmd == LINE_BARCODE_CMD_RIGHT) &&
            ctl->status.state != MOTION_STATE_IDLE &&
            ctl->status.state != MOTION_STATE_STOPPING) {
            bool left = turn_cmd == LINE_BARCODE_CMD_LEFT;   /* left pivot: right wheel drives */
            int32_t moved = left ? r - pivot_r0 : l - pivot_l0;
            float arc = 90.0f * (left ? ctl->cfg.right_turn_ticks_per_degree :
                                         ctl->cfg.left_turn_ticks_per_degree);
            if (moved < 0) moved = -moved;
            if ((float)moved >= arc * (left ? S2_PIVOT_LINE_MIN_FRACTION_LEFT :
                                              S2_PIVOT_LINE_MIN_FRACTION) &&
                line_service_snapshot(&msg) >= E_OK &&
                (msg.data[0] & LINE_BARCODE_CENTRE) != 0u) {
                LOG_INFO(TAG_S2_ROBOT, "pivot stopped on the line at %d%% of the arc",
                          (int)(100.0f * (float)moved / arc));
                vc_stop_motors(ctl, now, l, r);
                motion_control_step(ctl, now, l, r); /* STOPPING -> IDLE */
            }
        }
        if ((uint32_t)(now - turn_started) > S2_ROBOT_TURN_TIMEOUT_MS) {
            halt(ctl, now, l, r, "turn timeout");
        } else if (ctl->status.state == MOTION_STATE_IDLE && ctl->status.motion_complete) {
            if (turn_stage == TURN_ADVANCE && turn_cmd != LINE_BARCODE_CMD_STRAIGHT) {
                /* #8: on the crossing now; pivot as all_test does. */
                cmd.type = turn_cmd == LINE_BARCODE_CMD_RIGHT ?
                           MOTION_CMD_TURN_RIGHT_DEG : MOTION_CMD_TURN_LEFT_DEG;
                cmd.value = turn_cmd == LINE_BARCODE_CMD_UTURN ? 180.0f : 90.0f;
                motion_control_set_profile(ctl, turn_cmd == LINE_BARCODE_CMD_UTURN ?
                                           MOTION_PROFILE_UTURN : MOTION_PROFILE_JUNCTION);
                turn_stage = TURN_PIVOT;
                uturn_r0 = r;
                pivot_l0 = l;
                pivot_r0 = r;
                if (motion_control_command(ctl, &cmd, now, l, r) != MOTION_RESULT_OK)
                    halt(ctl, now, l, r, "turn rejected");
                return;
            }
            turning = false;
            turn_stage = 0u;
            er = line_service_enable(true);
            resumed_at = now;
            line_unsafe_active = false;
            /* #8: the sensors may still be on the crossing just handled. */
            leaving_junction = straddle;
            junction_done = now;
            steer_action = 0u;
            if (er < E_OK) halt(ctl, now, l, r, "line resume failed");
        }
        return;
    }
    /* Permit two sensor periods after enable/reset, with zero motor output. */
    if ((uint32_t)(now - resumed_at) < 30u) return;
    if (straddle) {
        straddle_step(ctl, now, l, r);
        return;
    }

    /* A newer snapshot fault must win over an older queued turn command. */
    er = line_service_snapshot(&msg);
    if (!line_snapshot_ok(ctl, now, l, r, er, &msg, false)) return;

    /* Drain events with a fixed budget so control cannot starve. */
    for (unsigned i = 0; i < 24u; ++i) {
        er = line_service_receive(&msg, TMO_POL);
        if (er == E_TMOUT) break;
        if (er < E_OK) {
            halt(ctl, now, l, r, "line receive failed");
            return;
        }
        uint8_t event = (msg.data[1] >> 3u) & 7u;
        if (event == LINE_BARCODE_EVT_BARCODE || event == LINE_BARCODE_EVT_COMMAND)
            last_letter = (msg.data[2] & LINE_BARCODE_LETTER_MASK) + 1u;
        if (event == LINE_BARCODE_EVT_JUNCTION) {
            /* #72 (decided 21 Nov): a junction without a mapped barcode before
               it goes straight, with or without an MQTT map (as all_test). */
            LOG_INFO(TAG_S2_ROBOT, "junction: %s, straight on",
                      map_received ? "no mapped barcode" : "no MQTT letter map yet");
            continue;
        }
        if (event == LINE_BARCODE_EVT_LOST || event == LINE_BARCODE_EVT_FAULT) {
            halt_line(ctl, now, l, r, "line lost or fault");
            return;
        }
        if (event == LINE_BARCODE_EVT_COMMAND) {
            unsigned action = msg.data[0] >> 6u;
            if (action == LINE_BARCODE_CMD_STRAIGHT) continue;
            er = line_service_enable(false);
            if (er < E_OK) {
                halt(ctl, now, l, r, "line pause failed");
                return;
            }
            cmd = (MOTION_COMMAND){MOTION_CMD_STOP, 0.0f};
            (void)motion_control_command(ctl, &cmd, now, l, r);
            motion_control_step(ctl, now, l, r); /* STOPPING -> IDLE */
            cmd.type = action == LINE_BARCODE_CMD_RIGHT ?
                       MOTION_CMD_TURN_RIGHT_DEG : MOTION_CMD_TURN_LEFT_DEG;
            cmd.value = action == LINE_BARCODE_CMD_UTURN ? 180.0f : 90.0f;
            motion_control_set_profile(ctl, MOTION_PROFILE_BASE); /* S3 model */
            if (motion_control_command(ctl, &cmd, now, l, r) != MOTION_RESULT_OK) {
                halt(ctl, now, l, r, "turn rejected");
                return;
            }
            turning = true;
            turn_started = now;
            return;
        }
    }
    er = line_service_snapshot(&msg);
    if (!line_snapshot_ok(ctl, now, l, r, er, &msg, true)) return;
    unsigned pos = (msg.data[0] >> 3u) & 7u;
    cmd.type = MOTION_CMD_FOLLOW_LINE;
    /* S3's positions increase from far-left to far-right. */
    cmd.value = pos <= 4u ? ((float)pos - 2.0f) * 0.5f : 0.0f;
    if (motion_control_command(ctl, &cmd, now, l, r) != MOTION_RESULT_OK)
        halt(ctl, now, l, r, "follow command rejected");
}

void robot_control_step(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    IMU_DATA imu;
    /* One S4 read per step, shared by control, RESUME checks and telemetry. */
    have_imu = subsystem4_get_latest(&imu) >= E_OK;
    if (have_imu) {
        last_imu = imu;
        if (imu.sample_count != imu_count) {
            imu_count = imu.sample_count;
            last_imu_sample = now;
        }
    }
    poll_mapping();
    poll_commands(ctl, now, l, r);
    control_step(ctl, now, l, r);
    update_distance(ctl, l, r);
    report_telemetry(ctl, now);
    report_barcode_frame(now);
}

#elif S2_MOTION_DEMO && !APP_TEST

#if S2_PID_BENCH
volatile uint32_t s2_bench_stop_ms;   /* set when run 2 sends STOP */
/*
 * S2-U10 / S2-U11 floor bench (#18). Two 100 cm forward runs, 5 s apart.
 *   run 1 (U10): logs left-minus-right travel and PWM every 20 ms, so the
 *                straight-line PID's response can be read from the console.
 *   run 2 (U11): sends STOP through subsystem2_send_command() (the same path
 *                as the real STOP) S2_BENCH_STOP_AFTER_MS into the run.
 *                task_motion.c times the motors-off and wheels-still points.
 * Needs 1.2 m of clear floor ahead. Not part of any normal build.
 */
void robot_control_step(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    static const MOTION_COMMAND fwd = {MOTION_CMD_FORWARD_CM, 100.0f};
    static unsigned run;                 /* 0, 1: finished runs */
    static bool active, stop_sent, done;
    static uint32_t started, next_start, step_no;
    static bool first = true;
    MOTION_COMMAND stop = {MOTION_CMD_STOP, 0.0f};

    if (first) {   /* the kernel clock does not start at 0: count from here */
        first = false;
        next_start = now + S2_ROBOT_START_MS;
    }
    if (done || ctl->status.state == MOTION_STATE_FAULT) {
        return;
    }
    if (!active) {
        if ((int32_t)(now - next_start) >= 0 &&
            motion_control_command(ctl, &fwd, now, l, r) == MOTION_RESULT_OK) {
            active = true;
            stop_sent = false;
            started = now;
            step_no = 0u;
            LOG_INFO(TAG_S2_BENCH, "run %u (%s) start; cols: ms L R err_mm pwmL pwmR",
                      run + 1u, run == 0u ? "U10 PID" : "U11 STOP");
        }
        return;
    }
    {
        uint32_t t = now - started;
        if ((step_no++ & 1u) == 0u) {
            /* err > 0: the left wheel is ahead of the right. */
            float el = (float)(l - ctl->command_start_left_ticks) / ctl->cfg.left_ticks_per_cm;
            float er = (float)(r - ctl->command_start_right_ticks) / ctl->cfg.right_ticks_per_cm;
            LOG_INFO(TAG_S2_BENCH, "%lu %ld %ld %ld %d %d", (unsigned long)t,
                      (long)l, (long)r, (long)((el - er) * 10.0f),
                      (int)ctl->status.left_pwm_percent,
                      (int)ctl->status.right_pwm_percent);
        }
        if (run == 1u && !stop_sent && t >= S2_BENCH_STOP_AFTER_MS) {
            stop_sent = true;
            LOG_INFO(TAG_S2_BENCH, "STOP sent at %lu ms", (unsigned long)t);
            s2_bench_stop_ms = now ? now : 1u;
            (void)subsystem2_send_command(&stop);
        }
        if (ctl->status.state == MOTION_STATE_IDLE || t > S2_DEMO_TIMEOUT_MS ||
            (stop_sent && t > S2_BENCH_STOP_AFTER_MS + 500u)) {
            LOG_INFO(TAG_S2_BENCH, "run %u end at %lu ms, state=%d", run + 1u,
                      (unsigned long)t, (int)ctl->status.state);
            active = false;
            next_start = now + 5000u;
            if (++run == 2u) {
                done = true;
                (void)motion_control_command(ctl, &stop, now, l, r);
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
void robot_control_step(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    static const MOTION_COMMAND seq[] = {
        {MOTION_CMD_TURN_RIGHT_DEG, 90.0f},  {MOTION_CMD_TURN_RIGHT_DEG, 90.0f},
        {MOTION_CMD_TURN_LEFT_DEG, 90.0f},   {MOTION_CMD_TURN_LEFT_DEG, 90.0f},
        {MOTION_CMD_TURN_RIGHT_DEG, 360.0f}, {MOTION_CMD_TURN_LEFT_DEG, 360.0f}
    };
    static unsigned stage;
    static bool waiting, done;
    static uint32_t started, pause_until, last_warn, last_beat;
    static bool banner;
    MOTION_COMMAND stop = {MOTION_CMD_STOP, 0.0f};

    if (!banner) {
        banner = true;
        /* tk_get_tim() does not start at 0 at boot (it read 3.35e9 ms), so
           the start delay counts from the first call, not from time 0. */
        pause_until = now + S2_ROBOT_START_MS;
        LOG_INFO(TAG_S2_BENCH, "pivot bench: first turn in %u ms, state=%d",
                  (UINT)S2_ROBOT_START_MS, (int)ctl->status.state);
    }
    if ((uint32_t)(now - last_beat) >= 3000u) {
        last_beat = now;
        LOG_INFO(TAG_S2_BENCH, "beat now=%lu next=%lu waiting=%d done=%d stage=%u state=%d",
                  (unsigned long)now, (unsigned long)pause_until, (int)waiting,
                  (int)done, stage, (int)ctl->status.state);
    }
    if (done) {
        (void)motion_control_command(ctl, &stop, now, l, r);
        return;
    }
    if (ctl->status.state == MOTION_STATE_FAULT) return;
    if (waiting) {
        if (ctl->status.state == MOTION_STATE_IDLE && ctl->status.motion_complete) {
            LOG_INFO(TAG_S2_BENCH, "turn %u (%s %d) done; ticks since start dL=%ld dR=%ld. Mark the heading now",
                      stage + 1u,
                      seq[stage].type == MOTION_CMD_TURN_RIGHT_DEG ? "right" : "left",
                      (int)seq[stage].value,
                      (long)(l - ctl->command_start_left_ticks),
                      (long)(r - ctl->command_start_right_ticks));
            waiting = false;
            pause_until = now + 8000u;
            if (++stage == sizeof(seq) / sizeof(seq[0])) {
                done = true;
                LOG_INFO(TAG_S2_BENCH, "finished; motors OFF");
            }
        } else if ((uint32_t)(now - started) > S2_DEMO_TIMEOUT_MS) {
            (void)motion_control_command(ctl, &stop, now, l, r);
            done = true;
            LOG_WARN(TAG_S2_BENCH, "timeout; motors OFF");
        }
    } else if ((int32_t)(now - pause_until) >= 0) {
        MOTION_RESULT res = motion_control_command(ctl, &seq[stage], now, l, r);
        if (res == MOTION_RESULT_OK) {
            waiting = true;
            started = now;
            LOG_INFO(TAG_S2_BENCH, "turn %u starting", stage + 1u);
        } else if ((uint32_t)(now - last_warn) >= 2000u) {
            last_warn = now;
            LOG_WARN(TAG_S2_BENCH, "turn %u rejected: result=%d state=%d", stage + 1u,
                      (int)res, (int)ctl->status.state);
        }
    }
}
#else

/* Standalone closed-loop sequence uses the same motor task and encoders. */
void robot_control_step(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    static const MOTION_COMMAND sequence[] = {
        {MOTION_CMD_FORWARD_CM, 20.0f},
        {MOTION_CMD_TURN_LEFT_DEG, 90.0f},
        {MOTION_CMD_FORWARD_CM, 20.0f},
        {MOTION_CMD_TURN_RIGHT_DEG, 90.0f}
    };
    static unsigned stage;
    static bool waiting, done;
    static uint32_t started, pause_until = S2_ROBOT_START_MS;
    MOTION_COMMAND stop = {MOTION_CMD_STOP, 0.0f};
    if (done) {
        (void)motion_control_command(ctl, &stop, now, l, r);
        return;
    }
    if (ctl->status.state == MOTION_STATE_FAULT) return;
    if (waiting) {
        if (ctl->status.state == MOTION_STATE_IDLE && ctl->status.motion_complete) {
            waiting = false;
            pause_until = now + 1500u;
            if (++stage == sizeof(sequence) / sizeof(sequence[0])) {
                done = true;
                LOG_INFO(TAG_S2_DEMO, "finished; motors OFF");
            }
        } else if ((uint32_t)(now - started) > S2_DEMO_TIMEOUT_MS) {
            (void)motion_control_command(ctl, &stop, now, l, r);
            done = true;
            LOG_WARN(TAG_S2_DEMO, "timeout; motors OFF");
        }
    } else if ((int32_t)(now - pause_until) >= 0) {
        if (motion_control_command(ctl, &sequence[stage], now, l, r) == MOTION_RESULT_OK) {
            waiting = true;
            started = now;
            LOG_INFO(TAG_S2_DEMO, "stage %u started", stage + 1u);
        }
    }
}
#endif /* S2_PID_BENCH / S2_PIVOT_BENCH */
#else
void robot_control_step(MOTION_CONTROLLER *ctl, uint32_t now, int32_t l, int32_t r)
{
    (void)ctl; (void)now; (void)l; (void)r;
}
#endif
