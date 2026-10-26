/** @file line_service.c
 * @brief IR GPIO ownership, periodic sampling and event transport.
 *
 * BEFORE version: each control flag is its own bool (no flag bitmask).
 */
#include "line_service.h"

#if TM_CONSOLE_UART
#error "IR_RIGHT uses GP0: select CONSOLE=usb_cdc CRASH_REPORT=0"
#endif

ID g_h_line_nav_q = 0;
static ID g_h_line_mutex = 0;
static ID g_h_line_task = 0;
static ID g_h_line_cycle = 0;

/* All of the following are protected by g_h_line_mutex. Only the sensor
 * task owns the processing context and accesses IR GPIO.
 */
static bool g_b_calibrated = false;
static bool g_b_enabled = false;
static bool g_b_reset = false;
static bool g_b_queue_fault = false;
static bool g_b_process_fault = false;
static line_barcode_sensors_t g_black_raw = {false, false, false};
static line_barcode_layout_t g_layout = LINE_BARCODE_LAYOUT_SIDE;
static line_barcode_map_t g_letter_map;
/* Letter read from the last barcode, waiting for the next junction. */
static line_barcode_nav_t g_nav = {false, '\0'};
static uint32_t g_sample_ms = 0u;
static line_barcode_msg_t g_latest;

static line_barcode_msg_t paused_msg(void)
{
    static line_barcode_sensors_t const no_sensors = {false, false, false};
    return (line_barcode_make_msg(no_sensors, LINE_BARCODE_POS_NONE,
                                  LINE_BARCODE_PAUSED,
                                  LINE_BARCODE_EVT_STATUS,
                                  LINE_BARCODE_ERR_NONE));
}

ER line_service_init(void)
{
    T_CMTX mutex_cfg = {0};
    T_CMBF queue_cfg = {0};
    ER result = E_OK;
    line_barcode_map_clear(&g_letter_map);
    g_latest = paused_msg();
    mutex_cfg.mtxatr = TA_INHERIT;
    g_h_line_mutex = tk_cre_mtx(&mutex_cfg);
    if (g_h_line_mutex < E_OK)
    {
        result = g_h_line_mutex;
    }
    else
    {
        queue_cfg.mbfatr = TA_TFIFO;
        queue_cfg.bufsz = LINE_SERVICE_QUEUE_BYTES;
        queue_cfg.maxmsz = sizeof(line_barcode_msg_t);
        g_h_line_nav_q = tk_cre_mbf(&queue_cfg);
        if (g_h_line_nav_q < E_OK)
        {
            result = g_h_line_nav_q;
            (void)tk_del_mtx(g_h_line_mutex);
            g_h_line_mutex = 0;
        }
    }
    return (result);
}

ER line_service_calibrate(line_barcode_sensors_t black_raw,
                          line_barcode_sensors_t white_raw)
{
    ER result = E_PAR;
    if ((black_raw.b_left != white_raw.b_left)
        && (black_raw.b_centre != white_raw.b_centre)
        && (black_raw.b_right != white_raw.b_right))
    {
        result = tk_loc_mtx(g_h_line_mutex, TMO_FEVR);
        if (E_OK == result)
        {
            g_black_raw = black_raw;
            g_b_calibrated = true;
            g_b_enabled = false;
            g_b_reset = true;
            g_b_queue_fault = false;
            g_b_process_fault = false;
            line_barcode_nav_clear(&g_nav);
            g_latest = paused_msg();
            result = tk_unl_mtx(g_h_line_mutex);
        }
    }
    return (result);
}

ER line_service_enable(bool b_enable)
{
    ER result = tk_loc_mtx(g_h_line_mutex, TMO_FEVR);
    if (E_OK == result)
    {
        if (b_enable && !g_b_calibrated)
        {
            result = E_OBJ;
        }
        else
        {
            g_b_enabled = b_enable;
            g_b_reset = true;
            g_b_queue_fault = false;
            g_b_process_fault = false;
            line_barcode_msg_t discarded;
            while (tk_rcv_mbf(g_h_line_nav_q, &discarded, TMO_POL) > 0)
            {
                /* Discard events from the preceding navigation session. */
            }
            g_latest = paused_msg();
        }
        ER unlock_result = tk_unl_mtx(g_h_line_mutex);
        result = (E_OK == result) ? unlock_result : result;
    }
    return (result);
}

ER line_service_set_layout(line_barcode_layout_t layout)
{
    ER result = E_PAR;
    if (layout <= LINE_BARCODE_LAYOUT_SIDE)
    {
        result = tk_loc_mtx(g_h_line_mutex, TMO_FEVR);
        if (E_OK == result)
        {
            result = g_b_enabled ? E_OBJ : E_OK;
            if (E_OK == result)
            {
                g_layout = layout;
                g_b_reset = true;
            }
            ER unlock_result = tk_unl_mtx(g_h_line_mutex);
            result = (E_OK == result) ? unlock_result : result;
        }
    }
    return (result);
}

ER line_service_map_letter(char letter, line_barcode_cmd_t command)
{
    ER result = tk_loc_mtx(g_h_line_mutex, TMO_FEVR);
    if (E_OK == result)
    {
        bool b_valid = line_barcode_map_set(&g_letter_map, letter, command);
        result = tk_unl_mtx(g_h_line_mutex);
        result = b_valid ? result : E_PAR;
    }
    return (result);
}

ER line_service_set_mapping(char left, char right, char straight, char uturn)
{
    char const letters[4] = {left, right, straight, uturn};
    line_barcode_map_t mapping;
    bool b_valid = true;
    line_barcode_map_clear(&mapping);

    /* Build the complete map first, so a bad message changes nothing. */
    for (uint8_t cmd = 0u; cmd < 4u; cmd++)
    {
        b_valid = b_valid && line_barcode_map_set(&mapping, letters[cmd],
                                                  (line_barcode_cmd_t)cmd);
        for (uint8_t other = 0u; other < cmd; other++)
        {
            b_valid = b_valid && (letters[other] != letters[cmd]);
        }
    }
    ER result = E_PAR;
    if (b_valid)
    {
        result = tk_loc_mtx(g_h_line_mutex, TMO_FEVR);
        if (E_OK == result)
        {
            g_letter_map = mapping;
            result = tk_unl_mtx(g_h_line_mutex);
        }
    }
    return (result);
}

ER line_service_clear_map(void)
{
    ER result = tk_loc_mtx(g_h_line_mutex, TMO_FEVR);
    if (E_OK == result)
    {
        line_barcode_map_clear(&g_letter_map);
        result = tk_unl_mtx(g_h_line_mutex);
    }
    return (result);
}

static ER snapshot_locked(line_barcode_msg_t *p_msg)
{
    SYSTIM now = {0};
    ER result = tk_get_tim(&now);
    *p_msg = g_latest;
    if ((E_OK == result) && ((now.lo - g_sample_ms) > 30u))
    {
        result = E_TMOUT;
    }
    return (result);
}

ER line_service_snapshot(line_barcode_msg_t *p_msg)
{
    ER result = E_PAR;
    if (NULL != p_msg)
    {
        result = tk_loc_mtx(g_h_line_mutex, TMO_FEVR);
        if (E_OK == result)
        {
            result = snapshot_locked(p_msg);
            ER unlock_result = tk_unl_mtx(g_h_line_mutex);
            result = (E_OK == result) ? unlock_result : result;
        }
    }
    return (result);
}

ER line_service_receive(line_barcode_msg_t *p_msg, TMO timeout)
{
    ER result = E_PAR;
    if (NULL != p_msg)
    {
        INT length = tk_rcv_mbf(g_h_line_nav_q, p_msg, timeout);
        result = (length < E_OK) ? length : E_SYS;
        result = (sizeof(*p_msg) == (uint32_t)length) ? E_OK : result;
    }
    return (result);
}

/* Reads each IR pin and converts it to "is black" using calibration. */
static line_barcode_sensors_t read_sensors(void)
{
    line_barcode_sensors_t sensors;
    bool b_left_high = (0 != gpio_get_val(LINE_SERVICE_PIN_LEFT));
    bool b_centre_high = (0 != gpio_get_val(LINE_SERVICE_PIN_CENTRE));
    bool b_right_high = (0 != gpio_get_val(LINE_SERVICE_PIN_RIGHT));
    sensors.b_left = (b_left_high == g_black_raw.b_left);
    sensors.b_centre = (b_centre_high == g_black_raw.b_centre);
    sensors.b_right = (b_right_high == g_black_raw.b_right);
    return (sensors);
}

static void publish_sample(line_barcode_msg_t msg)
{
    if (g_b_process_fault)
    {
        return;
    }
    (void)line_barcode_nav_update(&g_nav, &g_letter_map, &msg);
    if (LINE_BARCODE_EVT_STATUS != msg.event)
    {
        ER result = tk_snd_mbf(g_h_line_nav_q, &msg, sizeof(msg), TMO_POL);
        if (E_OK != result)
        {
            g_b_queue_fault = true;
        }
    }
    if (g_b_queue_fault)
    {
        msg.state = LINE_BARCODE_FAULT;
        msg.event = LINE_BARCODE_EVT_FAULT;
        msg.fault = LINE_BARCODE_ERR_QUEUE;
    }
    g_latest = msg;
    if (LINE_BARCODE_FAULT == msg.state)
    {
        g_b_process_fault = true;
    }
}

/* Lock covers control/reset, processing and publication, so pause cannot
 * race with an in-flight sample. No blocking queue send or console output.
 */
static void line_service_tick(void *p_extra)
{
    (void)p_extra;
    (void)tk_wup_tsk(g_h_line_task);
}

static void line_service_task(INT start_code, void *p_extra)
{
    line_barcode_ctx_t context;
    SYSTIM now = {0};
    uint32_t previous_ms = 0u;
    ER result = gpio_set_pin(LINE_SERVICE_PIN_LEFT, GPIO_MODE_IN);
    (void)start_code;
    (void)p_extra;
    if (E_OK == result)
    {
        result = gpio_set_pin(LINE_SERVICE_PIN_CENTRE, GPIO_MODE_IN);
    }
    if (E_OK == result)
    {
        result = gpio_set_pin(LINE_SERVICE_PIN_RIGHT, GPIO_MODE_IN);
    }
    line_barcode_reset(&context);
    (void)tk_get_tim(&now);
    previous_ms = now.lo;
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
        (void)tk_can_wup(TSK_SELF);
        (void)tk_get_tim(&now);
        uint32_t elapsed_ms = now.lo - previous_ms;
        previous_ms = now.lo;
        ER lock_result = tk_loc_mtx(g_h_line_mutex, TMO_FEVR);
        if (E_OK == lock_result)
        {
            if (g_b_reset)
            {
                (void)line_barcode_set_layout(&context, g_layout);
                g_b_reset = false;
            }
            g_sample_ms = now.lo;
            line_barcode_sensors_t sensors = read_sensors();
            line_barcode_msg_t msg = paused_msg();
            msg.sensors = sensors;
            if ((E_OK == result) && g_b_enabled)
            {
                uint16_t elapsed = (elapsed_ms > UINT16_MAX)
                                   ? UINT16_MAX : (uint16_t)elapsed_ms;
                msg = line_barcode_step(&context, sensors, elapsed);
            }
            if (E_OK != result)
            {
                msg = line_barcode_make_msg(sensors, LINE_BARCODE_POS_NONE,
                    LINE_BARCODE_FAULT, LINE_BARCODE_EVT_FAULT,
                    LINE_BARCODE_ERR_LINE);
            }
            publish_sample(msg);
            (void)tk_unl_mtx(g_h_line_mutex);
        }
    }
}

static ER start_sampling(void)
{
    T_CCYC cycle_cfg = {0};
    ER result = tk_sta_tsk(g_h_line_task, 0);
    if (E_OK == result)
    {
        cycle_cfg.cycatr = TA_HLNG | TA_STA;
        cycle_cfg.cychdr = line_service_tick;
        cycle_cfg.cyctim = LINE_BARCODE_PERIOD_MS;
        g_h_line_cycle = tk_cre_cyc(&cycle_cfg);
        if (g_h_line_cycle < E_OK)
        {
            result = g_h_line_cycle;
            (void)tk_ter_tsk(g_h_line_task);
        }
    }
    if (E_OK != result)
    {
        (void)tk_del_tsk(g_h_line_task);
        g_h_line_task = 0;
    }
    return (result);
}

ER line_service_start(void)
{
    ER result = E_OBJ;
    if ((g_h_line_mutex > 0) && (g_h_line_nav_q > 0)
        && (0 == g_h_line_task))
    {
        T_CTSK task_cfg = {0};
        task_cfg.task = line_service_task;
        task_cfg.itskpri = LINE_SERVICE_PRIORITY;
        task_cfg.stksz = TASK_STACK_SIZE;
        task_cfg.tskatr = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
        task_cfg.tskatr |= TA_ASSPRC;
        task_cfg.assprc = TP_PRC1;
#endif
        g_h_line_task = tk_cre_tsk(&task_cfg);
        result = (g_h_line_task > 0) ? start_sampling() : g_h_line_task;
    }
    return (result);
}

/*** end of file ***/
