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

enum { WAITING, SETTLING, RUNNING, PAUSED, STOPPED };
#define ALL_START_MS       5000u
#define ALL_READY_MS       15000u
#define ALL_IMU_STALE_MS   100u
#define ALL_LINE_READY_MS  500u
#define ALL_BASE_PWM       30.0f
#define ALL_INNER_PWM      15.0f

typedef struct {
    unsigned phase, terminal_state, barcode;
    uint32_t boot, started, settled, last_report, last_telemetry;
    uint32_t imu_count, imu_updated, imu_errors;
    uint32_t duration_ms, distance_mm;
    int32_t left_ticks, right_ticks;
    WHEEL_ODOMETRY odometry;
    const char *stop_reason;
    ER telemetry_error;
    bool started_once, force_status, imu_fault;
    IMU_DATA imu;
    line_barcode_msg_t line;
    ER imu_er, line_er;
    float left, right;
} ALL_TEST;

static void stop_test(ALL_TEST *t, const char *reason, unsigned state)
{
    drv_motor_stop();
    t->left = t->right = 0.0f;
    if (t->phase != STOPPED) {
        t->phase = STOPPED;
        t->terminal_state = state;
        t->stop_reason = reason;
        (void)line_service_enable(false);
        t->force_status = true;
        tm_printf((UB *)"[ALL TEST] STOPPED: %s; reboot to retry\n", reason);
    }
}

static bool imu_ready(const ALL_TEST *t, uint32_t now)
{
    return t->imu_er == E_OK && t->imu.sensor_ok && t->imu.calibrated &&
        t->imu_count != 0u && (uint32_t)(now - t->imu_updated) <= ALL_IMU_STALE_MS &&
        t->imu.motion_state != IMU_MOTION_TILT &&
        t->imu.motion_state != IMU_MOTION_COLLISION &&
        t->imu.motion_state != IMU_MOTION_FAULT &&
        t->imu.event != IMU_EVENT_TILT && t->imu.event != IMU_EVENT_COLLISION &&
        t->imu.event != IMU_EVENT_FAULT;
}

static void report_telemetry(ALL_TEST *t, uint32_t now)
{
    if (!t->force_status && (uint32_t)(now - t->last_telemetry) < 1000u) return;
    S1_TELEMETRY packet;
    unsigned state = t->phase == STOPPED ? t->terminal_state :
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
        switch (cmd.bytes[0]) {
        case S1_COMMAND_STATUS:
            t->force_status = true;
            break;
        case S1_COMMAND_PAUSE:
            if (t->phase != STOPPED) {
                drv_motor_stop();
                t->left = t->right = 0.0f;
                if (line_service_enable(false) < E_OK) {
                    stop_test(t, "pause failed", S1_STATE_FAULT);
                } else {
                    t->phase = PAUSED;
                    tm_printf((UB *)"[ALL TEST] MQTT pause\n");
                }
            }
            t->force_status = true;
            break;
        case S1_COMMAND_RESUME:
            if (t->phase == PAUSED && imu_ready(t, now) &&
                (uint32_t)(now - t->boot) >= ALL_START_MS) {
                if (line_service_enable(true) < E_OK) {
                    stop_test(t, "resume failed", S1_STATE_FAULT);
                } else {
                    t->phase = SETTLING;
                    t->settled = now;
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
}

/* One step owns motor outputs, S3 events and the S1 command queue. The task
 * keeps servicing MQTT/status after a stop; RESUME cannot clear a fault. */
static void all_test_step(ALL_TEST *t, uint32_t now)
{
    uint8_t travelled_mm;
    drv_encoder_get_counts(&t->left_ticks, &t->right_ticks);
    if (!wheel_odometry_step(&t->odometry, t->left_ticks, t->right_ticks,
                             t->phase == RUNNING, &travelled_mm)) {
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
    if ((t->phase == RUNNING || t->phase == SETTLING ||
         (t->phase == PAUSED && t->started_once)) &&
        (!imu_ready(t, now) || t->imu.read_errors != t->imu_errors)) {
        t->imu_fault = true;
        stop_test(t, "IMU unsafe, stale, or read error", S1_STATE_FAULT);
    }
    if (t->phase == WAITING) {
        if ((uint32_t)(now - t->boot) >= ALL_READY_MS) {
            t->imu_fault = true;
            stop_test(t, "IMU readiness timeout", S1_STATE_FAULT);
        } else if ((uint32_t)(now - t->boot) >= ALL_START_MS && imu_ready(t, now)) {
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
    if (t->phase == RUNNING) {
        if (t->line_er < E_OK) {
            stop_test(t, "line snapshot stale/failed", S1_STATE_FAULT);
        } else {
            for (unsigned i = 0u; i < 24u; ++i) {
                line_barcode_msg_t event;
                ER er = line_service_receive(&event, TMO_POL);
                if (er == E_TMOUT) break;
                if (er < E_OK) {
                    stop_test(t, "line queue failed", S1_STATE_FAULT); break;
                }
                unsigned code = (event.data[1] >> 3u) & 7u;
                if (code == LINE_BARCODE_EVT_LOST || code == LINE_BARCODE_EVT_FAULT ||
                    code == LINE_BARCODE_EVT_JUNCTION || code == LINE_BARCODE_EVT_COMMAND) {
                    stop_test(t, "line lost, fault, or junction", S1_STATE_FAULT); break;
                }
                if (code == LINE_BARCODE_EVT_BARCODE) {
                    unsigned letter = event.data[2] & LINE_BARCODE_LETTER_MASK;
                    t->barcode = letter == 0u ? S1_BARCODE_LEFT :
                        letter == 1u ? S1_BARCODE_RIGHT :
                        letter == 2u ? S1_BARCODE_STRAIGHT :
                        letter == 25u ? S1_BARCODE_UTURN : S1_BARCODE_NONE;
                    tm_printf((UB *)"[ALL TEST] barcode '%c' detected (no automatic turn in test)\n",
                              'A' + letter);
                }
            }
            unsigned state = t->line.data[1] & 7u;
            if (t->phase == RUNNING) {
                if (state == LINE_BARCODE_FAULT || state == LINE_BARCODE_SEARCH ||
                    state == LINE_BARCODE_PAUSED || state == LINE_BARCODE_JUNCTION) {
                    stop_test(t, "line lost, fault, or junction", S1_STATE_FAULT);
                } else if (line_follow_output(t->line.data[0], ALL_TEST_LINE_LAYOUT,
                                             ALL_BASE_PWM, ALL_INNER_PWM,
                                             &t->left, &t->right)) {
                    drv_motor_set(t->left, t->right);
                } else {
                    stop_test(t, "line absent or ambiguous L/C/R pattern", S1_STATE_FAULT);
                }
            }
        }
    }
    if ((uint32_t)(now - t->last_report) >= 500u) {
        unsigned sensors = t->line.data[0] & 7u;
        tm_printf((UB *)"[ALL TEST] phase=%u L=%u C=%u R=%u line_er=%d IMU ok=%d calibrated=%d state=%d samples=%lu PWM=%d/%d ticks=%ld/%ld height=%umm valid=%d stop=%s\n",
                  t->phase, (UINT)((sensors >> 2u) & 1u), (UINT)((sensors >> 1u) & 1u),
                  (UINT)(sensors & 1u), t->line_er, t->imu.sensor_ok,
                  t->imu.calibrated, t->imu.motion_state, (unsigned long)t->imu_count,
                  (int)t->left, (int)t->right, (long)t->left_ticks, (long)t->right_ticks,
                  (UINT)t->imu.ramp_height_mm, t->imu.ramp_height_valid,
                  t->stop_reason ? t->stop_reason : "none");
        t->last_report = now;
    }
    report_telemetry(t, now);
}

static void all_task(INT stacd, void *exinf)
{
    ALL_TEST test = {0};
    SYSTIM now;
    uint32_t last_control;
    (void)stacd;
    (void)exinf;
    tm_printf((UB *)"[ALL TEST] S1 MQTT + S2 motors + S3 line/barcode + S4 IMU\n"
              "[ALL TEST] keep stationary for IMU calibration; starts after 5s when ready\n"
              "[ALL TEST] 30%% PWM; GP7=left GP27=centre GP0=right; 1=black\n"
              "[ALL TEST] telemetry: unmeasured battery/speed/obstacle/scan fields=0 placeholders\n");
    tm_printf((UB *)"[ALL TEST] layout=%s\n",
              ALL_TEST_LINE_LAYOUT == LINE_BARCODE_LAYOUT_TRACK ?
              "TRACK (three line sensors; barcode decoding disabled)" : "SIDE (offset scanner)");
    if (tk_get_tim(&now) < E_OK) {
        drv_motor_stop();
        tk_slp_tsk(TMO_FEVR);
        return;
    }
    test.boot = now.lo;
    last_control = now.lo - LINE_BARCODE_PERIOD_MS;
    test.duration_ms = ALL_TEST_DURATION_MS;
    drv_encoder_get_counts(&test.left_ticks, &test.right_ticks);
    wheel_odometry_init(&test.odometry, S2_LEFT_TICKS_PER_CM,
                        S2_RIGHT_TICKS_PER_CM, test.left_ticks, test.right_ticks);
    if (!test.odometry.available)
        tm_printf((UB *)"[ALL TEST] height/distance unavailable: set measured S2_LEFT/RIGHT_TICKS_PER_CM; raw ticks are logged\n");
    tm_printf((UB *)"[ALL TEST] run limit=%lu ms (0=continuous while sensors are healthy)\n",
              (unsigned long)test.duration_ms);
    test.force_status = true;
    for (;;) {
        drv_encoder_poll();
        if (tk_get_tim(&now) < E_OK) {
            stop_test(&test, "clock failed", S1_STATE_FAULT);
            tk_slp_tsk(TMO_FEVR);
            return;
        }
        if ((uint32_t)(now.lo - last_control) >= LINE_BARCODE_PERIOD_MS) {
            all_test_step(&test, now.lo);
            last_control = now.lo;
        }
        if (tk_dly_tsk(S2_ENCODER_POLL_MS) < E_OK) {
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
    if (er == E_OK) er = subsystem4_init();
    if (er == E_OK) er = subsystem1_start();
    if (er == E_OK) er = line_service_start();
    if (er == E_OK) er = subsystem4_start();
    if (er == E_OK) {
        ID tid = app_start_task(all_task, 15, TP_PRC2, "all_buddies_test");
        er = tid < E_OK ? tid : E_OK;
    }
    if (er < E_OK) drv_motor_stop();
    return er;
}
#endif
