#include "robot_host_stubs.h"
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Inclusion makes adapter session state resettable between independent runs. */
#include "../../app_program/subsystem2/robot_control.c"

static IMU_DATA fake_imu;
static line_barcode_msg_t snapshot, queued;
static bool has_event, enabled;
static ER snapshot_error;
static unsigned checks;
#define EXPECT(x) do { assert(x); ++checks; } while (0)

int tm_printf(UB *format, ...) { (void)format; return 0; }
ER subsystem4_get_latest(IMU_DATA *out) { *out = fake_imu; return E_OK; }
ER line_service_calibrate(uint8_t b, uint8_t w) { return (b ^ w) == 7u ? E_OK : E_PAR; }
ER line_service_set_layout(uint8_t layout) { (void)layout; return E_OK; }
static char applied_map[5];   /* last map handed to S3, as "LRSU" */
ER line_service_set_mapping(char l, char r, char s, char u)
{
    applied_map[0] = l; applied_map[1] = r; applied_map[2] = s; applied_map[3] = u;
    return E_OK;
}
static unsigned fake_black, fake_white;
void line_service_barcode_debug(unsigned *black, unsigned *white, uint8_t *letter)
{ *black = fake_black; *white = fake_white; *letter = 0xffu; }
ER line_service_enable(bool on) { enabled = on; has_event = false; return E_OK; }
ER line_service_snapshot(line_barcode_msg_t *msg) { *msg = snapshot; return snapshot_error; }
ER line_service_receive(line_barcode_msg_t *msg, TMO timeout)
{
    (void)timeout;
    if (!has_event) return E_TMOUT;
    *msg = queued; has_event = false; return E_OK;
}

/* S1 command queue (#48): a FIFO the test fills; the VC polls it. */
static unsigned char commands[16];
static unsigned command_head, command_count, command_polls;
INT subsystem1_receive_command(S1_COMMAND *command, TMO timeout)
{
    (void)timeout;
    ++command_polls;
    if (command_count == 0u) return E_TMOUT;
    command->bytes[0] = commands[command_head++ % 16u];
    --command_count;
    return 1;
}
static void send_command(unsigned code)
{
    commands[(command_head + command_count++) % 16u] = (unsigned char)code;
}

/* S1 letter-map queue and telemetry sink (#72). */
static S1_MAPPING pending_map;
static bool has_map;
INT subsystem1_receive_mapping(S1_MAPPING *mapping, TMO timeout)
{
    (void)timeout;
    if (!has_map) return E_TMOUT;
    *mapping = pending_map; has_map = false;
    return (INT)S1_MAPPING_BYTES;
}
static void send_map(const char *letters) /* "LRSU", e.g. "FAZW" */
{
    unsigned char codes[S1_MAP_SLOTS];
    for (unsigned i = 0u; i < S1_MAP_SLOTS; ++i) codes[i] = (unsigned char)(letters[i] - 'A');
    EXPECT(s1_mapping_set(&pending_map, codes));
    has_map = true;
}
static S1_TELEMETRY published;
static unsigned publish_count;
ER subsystem1_publish_telemetry(const S1_TELEMETRY *packet)
{
    published = *packet; ++publish_count; return E_OK;
}
static int field(S1_FIELD f)
{
    int value = -1;
    EXPECT(s1_telemetry_get(&published, f, &value));
    return value;
}

/* S5 executor: modelled as idle (no obstacle, no start-up hold), so these
 * tests exercise the VC's own line/barcode/IMU logic. Avoidance hand-over is
 * not covered here. */
static unsigned s5_steps;
void s5_exec_set_following(S5_EXEC *p_x, BOOL b_following)
{ (void)p_x; (void)b_following; }
BOOL s5_exec_startup_hold(void) { return FALSE; }
BOOL s5_exec_step(S5_EXEC *p_x, const S5_EXEC_OPS *p_ops, void *p_ctx, UW now_ms)
{ (void)p_x; (void)p_ops; (void)p_ctx; (void)now_ms; ++s5_steps; return FALSE; }

static MOTION_CONFIG config(void)
{
    return (MOTION_CONFIG){
        .left_ticks_per_cm=10, .right_ticks_per_cm=10,
        .left_turn_ticks_per_degree=2, .right_turn_ticks_per_degree=2,
        .base_pwm_percent=50, .minimum_effective_pwm_percent=10,
        .maximum_pwm_percent=100, .distance_tolerance_cm=0.5f,
        .angle_tolerance_deg=1, .encoder_stall_timeout_ms=500,
        .kp=2, .integral_limit=100, .correction_limit=30
    };
}

static void reset(MOTION_CONTROLLER *ctl)
{
    MOTION_CONFIG cfg = config();
    armed = halted = turning = configured = false;
    last_imu_sample = imu_count = imu_errors = resumed_at = turn_started = 0;
    /* Controller state added after 2026-10-28 (line-stop debounce/re-arm, S5). */
    line_halt = line_unsafe_active = false;
    line_unsafe_since = rearm_since = 0;
    rearm_l = rearm_r = 0;
    memset(&vc_s5, 0, sizeof(vc_s5));
    /* #48: these tests run with auto-start unless a test turns it off. */
    auto_start = true;
    start_requested = ready_reported = false;
    command_head = command_count = command_polls = 0u;
    /* #72 state. */
    paused = map_received = status_requested = have_imu = in_avoid = false;
    odo_started = false;
    last_letter = 0u;
    last_telemetry = 0u;
    telemetry_error = E_OK;
    distance_cm = 0.0f;
    memset(&last_imu, 0, sizeof(last_imu));
    memset(map_letters, 0, sizeof(map_letters));
    memset(applied_map, 0, sizeof(applied_map));
    has_map = false;
    publish_count = 0u;
    fake_black = fake_white = 0u;
    has_event = enabled = false; snapshot_error = E_OK;
    memset(&fake_imu, 0, sizeof(fake_imu));
    fake_imu.sensor_ok = fake_imu.calibrated = TRUE;
    fake_imu.sample_count = 1;
    snapshot = (line_barcode_msg_t){{3u | (2u << 3u), LINE_BARCODE_FOLLOW, 0u}};
    EXPECT(motion_control_init(ctl, &cfg, 0, 0, 0));
}

static void follow(MOTION_CONTROLLER *ctl)
{
    reset(ctl);
    robot_control_step(ctl, 5000, 0, 0);
    EXPECT(armed && enabled);
    EXPECT(ctl->status.state == MOTION_STATE_IDLE);
    robot_control_step(ctl, 5030, 0, 0);
    motion_control_step(ctl, 5040, 1, 1);
    EXPECT(ctl->status.active_command == MOTION_CMD_FOLLOW_LINE);
    EXPECT(ctl->status.left_pwm_percent > 0);
}

static void stopped(MOTION_CONTROLLER *ctl, uint32_t now)
{
    robot_control_step(ctl, now, 1, 1);
    EXPECT(halted && !enabled);
    EXPECT(ctl->status.left_pwm_percent == 0 && ctl->status.right_pwm_percent == 0);
    fake_imu.motion_state = IMU_MOTION_NORMAL;
    fake_imu.sensor_ok = TRUE;
    fake_imu.sample_count++;
    robot_control_step(ctl, now + 1, 1, 1);
    EXPECT(halted && ctl->status.left_pwm_percent == 0);
}

/* A queued LOST event makes a recoverable line stop. */
static void line_stop(MOTION_CONTROLLER *ctl, uint32_t now)
{
    queued = snapshot;
    queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_LOST << 3u);
    has_event = true;
    robot_control_step(ctl, now, 1, 1);
    EXPECT(halted && line_halt && !enabled);
    EXPECT(ctl->status.left_pwm_percent == 0 && ctl->status.right_pwm_percent == 0);
}

/* Stale, searching or paused snapshots must persist for S2_LINE_UNSAFE_MS
 * before the VC stops (one late sample must not latch a stop). */
static void stopped_debounced(MOTION_CONTROLLER *ctl, uint32_t now)
{
    robot_control_step(ctl, now, 1, 1);
    EXPECT(!halted);
    /* Keep the IMU heartbeat fresh (S2_IMU_STALE_MS) so only the line
     * condition can cause the stop. */
    for (uint32_t t = now + 50u; t < now + S2_LINE_UNSAFE_MS; t += 50u) {
        fake_imu.sample_count++;
        robot_control_step(ctl, t, 1, 1);
        EXPECT(!halted);
    }
    fake_imu.sample_count++;
    stopped(ctl, now + S2_LINE_UNSAFE_MS);
}

int main(void)
{
    MOTION_CONTROLLER ctl;
    reset(&ctl);
    fake_imu.calibrated = FALSE;
    robot_control_step(&ctl, 6000, 0, 0);
    EXPECT(!armed && ctl.status.left_pwm_percent == 0);

    follow(&ctl);
    snapshot.data[0] = 3u | (0u << 3u);
    robot_control_step(&ctl, 5041, 1, 1);
    motion_control_step(&ctl, 5050, 2, 2);
    EXPECT(ctl.status.left_pwm_percent < ctl.status.right_pwm_percent);
    snapshot.data[0] = 3u | (4u << 3u);
    robot_control_step(&ctl, 5051, 2, 2);
    motion_control_step(&ctl, 5060, 3, 3);
    EXPECT(ctl.status.left_pwm_percent > ctl.status.right_pwm_percent);

    for (unsigned action = 0; action < 4; ++action) {
        follow(&ctl);
        queued = (line_barcode_msg_t){{(uint8_t)(action << 6u),
                    LINE_BARCODE_EVT_COMMAND << 3u, 0u}};
        has_event = true;
        robot_control_step(&ctl, 5041, 1, 1);
        if (action == LINE_BARCODE_CMD_STRAIGHT) {
            EXPECT(!turning && ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);
            continue;
        }
        EXPECT(turning && !enabled);
        EXPECT(ctl.target_value == (action == LINE_BARCODE_CMD_UTURN ? 180 : 90));
        bool right = action == LINE_BARCODE_CMD_RIGHT;
        EXPECT(ctl.status.active_command == (right ? MOTION_CMD_TURN_RIGHT_DEG : MOTION_CMD_TURN_LEFT_DEG));
        int ticks = (int)(ctl.target_value * 2) + 1;
        motion_control_step(&ctl, 5050, right ? ticks : 1, right ? 1 : ticks);
        robot_control_step(&ctl, 5051, right ? ticks : 1, right ? 1 : ticks);
        EXPECT(!turning && enabled);
    }
    follow(&ctl); fake_imu.motion_state = IMU_MOTION_COLLISION; stopped(&ctl, 5041);
    /* Tilt is report-only (#67): the robot keeps following. */
    follow(&ctl); fake_imu.motion_state = IMU_MOTION_TILT;
    fake_imu.event = IMU_EVENT_TILT;
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(!halted && enabled);
    EXPECT(ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);
    follow(&ctl); fake_imu.event = IMU_EVENT_FAULT; stopped(&ctl, 5041);
    follow(&ctl); fake_imu.sensor_ok = FALSE; stopped(&ctl, 5041);
    follow(&ctl); fake_imu.read_errors++; stopped(&ctl, 5041);
    follow(&ctl); stopped(&ctl, 5101); /* stale IMU heartbeat */
    follow(&ctl); snapshot_error = E_TMOUT; stopped_debounced(&ctl, 5041);
    follow(&ctl); snapshot.data[1] = LINE_BARCODE_FAULT; stopped(&ctl, 5041);
    follow(&ctl); snapshot.data[1] = LINE_BARCODE_SEARCH; stopped_debounced(&ctl, 5041);
    for (unsigned event = 0; event < 2; ++event) {
        const unsigned events[] = {LINE_BARCODE_EVT_LOST, LINE_BARCODE_EVT_FAULT};
        follow(&ctl); queued = snapshot;
        queued.data[1] = (uint8_t)(events[event] << 3u); has_event = true;
        stopped(&ctl, 5041);
    }
    /* #72 (decided 21 Nov): a junction with no mapped barcode before it goes
     * straight on, both before and after an MQTT map has arrived. */
    for (unsigned with_map = 0; with_map < 2; ++with_map) {
        follow(&ctl); queued = snapshot;
        queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_JUNCTION << 3u); has_event = true;
        map_received = with_map != 0u;
        robot_control_step(&ctl, 5041, 1, 1);
        EXPECT(!halted && enabled && !turning);
        robot_control_step(&ctl, 5042, 1, 1);
        motion_control_step(&ctl, 5050, 2, 2);
        EXPECT(ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);
    }
    /* Line stops re-arm by themselves after S2_LINE_REARM_MS of stable black
     * on centre + right with the wheels still (ported from the retired
     * all_buddies host tests, #9). */
    {
        const uint32_t t0 = 5041u, rearm = S2_LINE_REARM_MS;
        follow(&ctl); line_stop(&ctl, t0);
        robot_control_step(&ctl, t0 + 50u, 1, 1);
        robot_control_step(&ctl, t0 + rearm - 1u, 1, 1);
        EXPECT(halted && !enabled);                 /* not yet */
        robot_control_step(&ctl, t0 + rearm, 1, 1);
        EXPECT(!halted && enabled);                 /* re-armed on its own */

        /* A brief loss of black restarts the wait. */
        follow(&ctl); line_stop(&ctl, t0);
        snapshot.data[0] = 0u; robot_control_step(&ctl, 5500u, 1, 1);
        snapshot.data[0] = 3u | (2u << 3u);
        robot_control_step(&ctl, t0 + rearm, 1, 1);
        EXPECT(halted);
        robot_control_step(&ctl, 5500u + rearm, 1, 1);
        EXPECT(!halted && enabled);

        /* Moving the robot by hand (encoders change) restarts the wait. */
        follow(&ctl); line_stop(&ctl, t0);
        robot_control_step(&ctl, 5500u, 5, 5);
        robot_control_step(&ctl, t0 + rearm, 5, 5);
        EXPECT(halted);
        robot_control_step(&ctl, 5500u + rearm, 5, 5);
        EXPECT(!halted && enabled);

        /* An IMU (safety) stop never re-arms, even on stable black. */
        follow(&ctl); fake_imu.event = IMU_EVENT_FAULT;
        robot_control_step(&ctl, t0, 1, 1);
        EXPECT(halted && !line_halt);
        fake_imu.event = IMU_EVENT_NONE;
        for (uint32_t t = t0 + 100u; t <= t0 + 3u * rearm; t += 100u) {
            fake_imu.sample_count++;
            robot_control_step(&ctl, t, 1, 1);
        }
        EXPECT(halted && ctl.status.left_pwm_percent == 0);
    }
    follow(&ctl);
    queued = (line_barcode_msg_t){{0u, LINE_BARCODE_EVT_COMMAND << 3u, 0u}};
    has_event = true;
    snapshot.data[1] = LINE_BARCODE_FAULT;
    stopped(&ctl, 5041);
    EXPECT(!turning); /* Newer fault takes precedence over queued turn. */

    follow(&ctl);
    queued = (line_barcode_msg_t){{0u, LINE_BARCODE_EVT_COMMAND << 3u, 0u}};
    has_event = true;
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(turning);
    fake_imu.sample_count++;
    stopped(&ctl, 15042); /* Turn deadline with a fresh IMU sample. */

    follow(&ctl);
    fake_imu.motion_state = IMU_MOTION_HUMP;
    fake_imu.event = IMU_EVENT_HUMP;
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(!halted && ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);

    follow(&ctl);
    motion_control_step(&ctl, 5600, 1, 1);
    EXPECT(ctl.status.state == MOTION_STATE_FAULT);
    EXPECT(ctl.status.left_pwm_percent == 0 && ctl.status.right_pwm_percent == 0);

    /* The original standalone distance and turn interface remains usable. */
    reset(&ctl);
    MOTION_COMMAND cmd = {MOTION_CMD_FORWARD_CM, 20};
    EXPECT(motion_control_command(&ctl, &cmd, 0, 0, 0) == MOTION_RESULT_OK);
    motion_control_step(&ctl, 10, 200, 200);
    EXPECT(ctl.status.state == MOTION_STATE_IDLE && ctl.status.motion_complete);
    cmd = (MOTION_COMMAND){MOTION_CMD_TURN_RIGHT_DEG, 90};
    EXPECT(motion_control_command(&ctl, &cmd, 20, 200, 200) == MOTION_RESULT_OK);
    motion_control_step(&ctl, 30, 380, 200);
    EXPECT(ctl.status.state == MOTION_STATE_IDLE);
    cmd = (MOTION_COMMAND){MOTION_CMD_FORWARD_CM, NAN};
    EXPECT(motion_control_command(&ctl, &cmd, 40, 380, 200) == MOTION_RESULT_INVALID_COMMAND);
    /* #48 Ready state: with auto-start off, an initialised robot waits,
     * motors off, until START. Other commands do not start it. */
    reset(&ctl);
    auto_start = false;
    robot_control_step(&ctl, 5000, 0, 0);
    EXPECT(!armed && !enabled && ready_reported);
    send_command(S1_COMMAND_PAUSE);
    send_command(S1_COMMAND_RESUME);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5050, 0, 0);
    EXPECT(!armed && !enabled && command_count == 0u); /* drained, not a start */
    EXPECT(ctl.status.left_pwm_percent == 0 && ctl.status.right_pwm_percent == 0);
    send_command(S1_COMMAND_START);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5100, 0, 0);
    EXPECT(armed && enabled && start_requested);
    robot_control_step(&ctl, 5130, 0, 0);
    motion_control_step(&ctl, 5140, 1, 1);
    EXPECT(ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);
    send_command(S1_COMMAND_START);                       /* already running */
    robot_control_step(&ctl, 5150, 1, 1);
    EXPECT(armed && command_count == 0u);

    /* A START sent during initialisation is kept until the robot is Ready. */
    reset(&ctl);
    auto_start = false;
    send_command(S1_COMMAND_START);
    robot_control_step(&ctl, 1000, 0, 0);                  /* before S2_ROBOT_START_MS */
    EXPECT(!armed && start_requested && !ready_reported);
    fake_imu.sample_count++;
    robot_control_step(&ctl, S2_ROBOT_START_MS, 0, 0);
    EXPECT(armed && enabled);

    /* Polling never blocks and drains at most S1_QUEUE_DEPTH per step. */
    reset(&ctl);
    auto_start = false;
    for (unsigned i = 0u; i < 10u; ++i) send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 1000, 0, 0);
    EXPECT(command_count == 10u - S1_QUEUE_DEPTH && command_polls == S1_QUEUE_DEPTH);

    /* ---- #72 MQTT link ------------------------------------------------- */

    /* PAUSE stops at once and holds; RESUME on a healthy line and IMU
     * restarts line following. */
    follow(&ctl);
    send_command(S1_COMMAND_PAUSE);
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(paused && !enabled && !halted);
    EXPECT(ctl.status.left_pwm_percent == 0 && ctl.status.right_pwm_percent == 0);
    for (uint32_t t = 5100u; t <= 7000u; t += 100u) {
        fake_imu.sample_count++;
        robot_control_step(&ctl, t, 1, 1);
        motion_control_step(&ctl, t, 1, 1);
        EXPECT(paused && ctl.status.left_pwm_percent == 0);
    }
    send_command(S1_COMMAND_RESUME);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 7050, 1, 1);
    EXPECT(!paused && enabled);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 7100, 1, 1);
    motion_control_step(&ctl, 7110, 2, 2);
    EXPECT(ctl.status.active_command == MOTION_CMD_FOLLOW_LINE);

    /* RESUME is refused off the line or with an unhealthy IMU. */
    follow(&ctl);
    send_command(S1_COMMAND_PAUSE);
    robot_control_step(&ctl, 5041, 1, 1);
    snapshot.data[0] = 0u;                                 /* off the line */
    send_command(S1_COMMAND_RESUME);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5060, 1, 1);
    EXPECT(paused && !enabled);
    snapshot.data[0] = 3u | (2u << 3u);
    fake_imu.sensor_ok = FALSE;
    send_command(S1_COMMAND_RESUME);
    robot_control_step(&ctl, 5070, 1, 1);
    EXPECT(paused && !enabled);

    /* PAUSE during a line stop blocks the automatic re-arm until RESUME. */
    {
        const uint32_t t0 = 5041u, rearm = S2_LINE_REARM_MS;
        follow(&ctl); line_stop(&ctl, t0);
        send_command(S1_COMMAND_PAUSE);
        robot_control_step(&ctl, t0 + 10u, 1, 1);
        robot_control_step(&ctl, t0 + 3u * rearm, 1, 1);
        EXPECT(halted && paused && !enabled);              /* no re-arm while paused */
        send_command(S1_COMMAND_RESUME);
        fake_imu.sample_count++;
        robot_control_step(&ctl, t0 + 3u * rearm + 1u, 1, 1);
        EXPECT(!paused && halted);          /* time paused doesn't count */
        robot_control_step(&ctl, t0 + 4u * rearm, 1, 1);
        EXPECT(halted && !enabled);
        robot_control_step(&ctl, t0 + 4u * rearm + 1u, 1, 1);
        EXPECT(!halted && enabled);         /* a full re-arm wait after RESUME */
    }

    /* A safety stop ignores PAUSE and RESUME: it needs a reboot. */
    follow(&ctl); fake_imu.event = IMU_EVENT_FAULT;
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(halted && !line_halt);
    fake_imu.event = IMU_EVENT_NONE;
    send_command(S1_COMMAND_PAUSE);
    send_command(S1_COMMAND_RESUME);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5100, 1, 1);
    EXPECT(halted && !line_halt && !paused && !enabled);

    /* PAUSE in Ready cancels a pending START. */
    reset(&ctl);
    auto_start = false;
    send_command(S1_COMMAND_START);
    robot_control_step(&ctl, 1000, 0, 0);
    send_command(S1_COMMAND_PAUSE);
    robot_control_step(&ctl, 1100, 0, 0);
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5000, 0, 0);
    EXPECT(!armed && !start_requested);

    /* The MQTT letter map is applied to S3; a later barcode and its mapped
     * direction appear in telemetry. A bad map changes nothing. */
    follow(&ctl);
    EXPECT(applied_map[0] == '\0' && !map_received);        /* no built-in map */
    send_map("FAZW");
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(map_received && strcmp(applied_map, "FAZW") == 0);
    pending_map.bytes[0] = pending_map.bytes[1] = pending_map.bytes[2] = 0u; /* A,A,A,A */
    has_map = true;
    robot_control_step(&ctl, 5042, 1, 1);
    EXPECT(strcmp(applied_map, "FAZW") == 0);               /* duplicates rejected */
    queued = snapshot;
    queued.data[1] = (uint8_t)(LINE_BARCODE_EVT_BARCODE << 3u);
    queued.data[2] = 'Z' - 'A';
    has_event = true;
    fake_imu.sample_count++;
    robot_control_step(&ctl, 5060, 1, 1);
    EXPECT(last_letter == 26u);
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 5061, 1, 1);
    EXPECT(field(S1_FIELD_BARCODE_LETTER) == 26 &&
           field(S1_FIELD_BARCODE) == S1_BARCODE_STRAIGHT); /* Z = straight */
    EXPECT(field(S1_FIELD_MAP_RECEIVED) == 1);             /* #38 */

    /* Telemetry: once a second, STATUS forces one at once; state, line bits,
     * distance, hump and barcode counts come from the live sources. */
    reset(&ctl);
    auto_start = false;
    robot_control_step(&ctl, 1000, 0, 0);
    EXPECT(publish_count == 1u && field(S1_FIELD_STATE) == S1_STATE_IDLE);
    EXPECT(field(S1_FIELD_MAP_RECEIVED) == 0);             /* #38: waiting for map */
    robot_control_step(&ctl, 1500, 0, 0);
    EXPECT(publish_count == 1u);
    robot_control_step(&ctl, 2000, 0, 0);
    EXPECT(publish_count == 2u);
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 2001, 0, 0);
    EXPECT(publish_count == 3u);
    follow(&ctl);
    fake_imu.ramp_height_mm = 23u;
    fake_black = 15u; fake_white = 14u;
    snapshot.data[0] = 6u | (2u << 3u);                    /* S3 L+C black */
    fake_imu.sample_count++;
    robot_control_step(&ctl, 6000, 100, 100);              /* 10 ticks/cm: 10 cm */
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 6001, 100, 100);
    EXPECT(field(S1_FIELD_STATE) == S1_STATE_FOLLOWING);
    EXPECT(field(S1_FIELD_LINE_SENSORS) == 3);             /* wire: L bit0, C bit1 */
    EXPECT(field(S1_FIELD_DISTANCE_CM) == 10);
    EXPECT(field(S1_FIELD_HIGHEST_HUMP_MM) == 23);
    EXPECT(field(S1_FIELD_BARCODE_BLACK) == 15 && field(S1_FIELD_BARCODE_WHITE) == 14);
    EXPECT(field(S1_FIELD_FAULTS) == 0);
    send_command(S1_COMMAND_PAUSE);
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 6002, 100, 100);
    EXPECT(field(S1_FIELD_STATE) == S1_STATE_IDLE);        /* paused */
    follow(&ctl); line_stop(&ctl, 5041);
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 5042, 1, 1);
    EXPECT(field(S1_FIELD_STATE) == S1_STATE_RECOVERING);
    follow(&ctl); fake_imu.sensor_ok = FALSE;
    send_command(S1_COMMAND_STATUS);
    robot_control_step(&ctl, 5041, 1, 1);
    EXPECT(field(S1_FIELD_STATE) == S1_STATE_FAULT &&
           (field(S1_FIELD_FAULTS) & S1_FAULT_IMU) != 0);

    printf("robot host tests: %u checks passed\n", checks);
    return 0;
}
