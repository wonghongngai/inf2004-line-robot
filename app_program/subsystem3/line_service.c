/** @file line_service.c
 * @brief IR GPIO ownership, periodic sampling and compact event transport.
 */
#include "line_service.h"

#if TM_CONSOLE_UART
#error "IR_RIGHT uses GP0: select CONSOLE=usb_cdc CRASH_REPORT=0"
#endif

#define SERVICE_FLAG_CALIBRATED   1u
#define SERVICE_FLAG_ENABLED      2u
#define SERVICE_FLAG_RESET        4u
#define SERVICE_FLAG_QUEUE_FAULT  8u
#define SERVICE_FLAG_PROCESS_FAULT 16u

ID g_h_line_nav_q = 0;
static ID g_h_line_mutex = 0;
static ID g_h_line_task = 0;
static ID g_h_line_cycle = 0;
static uint8_t g_control_flags = 0u;
static uint8_t g_black_raw = 0u;
static uint8_t g_layout = LINE_BARCODE_LAYOUT_SIDE;
/* Packed per-letter validity/command entries, protected by mutex. */
static line_barcode_map_t g_letter_map = {{0u}};
/* Letter read from the last barcode, waiting for the next junction. */
static line_barcode_nav_t g_nav = {0u};
static uint32_t g_sample_ms = 0u;
static line_barcode_msg_t g_latest = {{0u, LINE_BARCODE_PAUSED}};

/* All shared packed fields and the three-byte snapshot use this mutex.
 * Only the sensor task owns the processing context and accesses IR GPIO.
 */
ER line_service_init(void)
{
    T_CMTX mutex_cfg = {0};
    T_CMBF queue_cfg = {0};
    ER result = E_OK;
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

ER line_service_calibrate(uint8_t black_raw, uint8_t white_raw)
{
    ER result = E_PAR;
    if (((black_raw | white_raw) <= LINE_BARCODE_SENSOR_MASK)
        && (LINE_BARCODE_SENSOR_MASK == (black_raw ^ white_raw)))
    {
        result = tk_loc_mtx(g_h_line_mutex, TMO_FEVR);
        if (E_OK == result)
        {
            g_black_raw = black_raw;
            g_control_flags = SERVICE_FLAG_CALIBRATED | SERVICE_FLAG_RESET;
            line_barcode_nav_clear(&g_nav);
            g_latest = line_barcode_pack(0u, 6u, 0u, LINE_BARCODE_PAUSED,
                                         LINE_BARCODE_EVT_STATUS, 0u);
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
        if (b_enable && (0u == (g_control_flags & SERVICE_FLAG_CALIBRATED)))
        {
            result = E_OBJ;
        }
        else
        {
            g_control_flags &= SERVICE_FLAG_CALIBRATED;
            g_control_flags |= SERVICE_FLAG_RESET;
            g_control_flags |= b_enable ? SERVICE_FLAG_ENABLED : 0u;
            line_barcode_msg_t discarded = {{0u, 0u}};
            while (tk_rcv_mbf(g_h_line_nav_q, &discarded, TMO_POL) > 0)
            {
                /* Discard events from the preceding navigation session. */
            }
            g_latest = line_barcode_pack(0u, 6u, 0u, LINE_BARCODE_PAUSED,
                                         LINE_BARCODE_EVT_STATUS, 0u);
        }
        ER unlock_result = tk_unl_mtx(g_h_line_mutex);
        result = (E_OK == result) ? unlock_result : result;
    }
    return (result);
}

ER line_service_set_layout(uint8_t layout)
{
    ER result = E_PAR;
    if (layout <= LINE_BARCODE_LAYOUT_SIDE)
    {
        result = tk_loc_mtx(g_h_line_mutex, TMO_FEVR);
        if (E_OK == result)
        {
            result = (0u != (g_control_flags & SERVICE_FLAG_ENABLED))
                     ? E_OBJ : E_OK;
            g_layout = (E_OK == result) ? layout : g_layout;
            g_control_flags |= (E_OK == result) ? SERVICE_FLAG_RESET : 0u;
            ER unlock_result = tk_unl_mtx(g_h_line_mutex);
            result = (E_OK == result) ? unlock_result : result;
        }
    }
    return (result);
}

ER line_service_map_letter(char letter, uint8_t command)
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
    line_barcode_map_t mapping = {{0u}};
    bool b_valid = true;

    /* Build the complete map first, so a bad message changes nothing. */
    for (uint8_t cmd = 0u; cmd < 4u; cmd++)
    {
        b_valid = b_valid && line_barcode_map_set(&mapping, letters[cmd], cmd);
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

static uint8_t read_sensors(void)
{
    uint8_t raw = (uint8_t)((gpio_get_val(LINE_SERVICE_PIN_LEFT) << 2u)
                            | (gpio_get_val(LINE_SERVICE_PIN_CENTRE) << 1u)
                            | gpio_get_val(LINE_SERVICE_PIN_RIGHT));
    return (raw);
}

static void publish_sample(line_barcode_msg_t msg)
{
    if (0u != (g_control_flags & SERVICE_FLAG_PROCESS_FAULT))
    {
        return;
    }
    (void)line_barcode_nav_update(&g_nav, &g_letter_map, &msg);
    uint8_t event = (msg.data[1] >> 3u) & 7u;
    if (LINE_BARCODE_EVT_STATUS != event)
    {
        ER result = tk_snd_mbf(g_h_line_nav_q, &msg, sizeof(msg), TMO_POL);
        if (E_OK != result)
        {
            g_control_flags |= SERVICE_FLAG_QUEUE_FAULT;
        }
    }
    if (0u != (g_control_flags & SERVICE_FLAG_QUEUE_FAULT))
    {
        msg.data[1] = (uint8_t)(LINE_BARCODE_FAULT
                                | (LINE_BARCODE_EVT_FAULT << 3u));
        msg.data[2] |= (uint8_t)(LINE_BARCODE_ERR_QUEUE
                                 << LINE_BARCODE_FAULT_SHIFT);
    }
    g_latest = msg;
    if (LINE_BARCODE_FAULT == (msg.data[1] & 7u))
    {
        g_control_flags |= SERVICE_FLAG_PROCESS_FAULT;
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
            if (0u != (g_control_flags & SERVICE_FLAG_RESET))
            {
                (void)line_barcode_set_layout(&context, g_layout);
                g_control_flags &= (uint8_t)~SERVICE_FLAG_RESET;
            }
            g_sample_ms = now.lo;
            uint8_t sensors = (uint8_t)(~(read_sensors() ^ g_black_raw) & 7u);
            line_barcode_msg_t msg = line_barcode_pack(sensors, 6u, 0u,
                LINE_BARCODE_PAUSED, LINE_BARCODE_EVT_STATUS, 0u);
            if ((E_OK == result)
                && (0u != (g_control_flags & SERVICE_FLAG_ENABLED)))
            {
                uint16_t elapsed = (elapsed_ms > UINT16_MAX)
                                   ? UINT16_MAX : (uint16_t)elapsed_ms;
                msg = line_barcode_step(&context, sensors, elapsed);
            }
            if (E_OK != result)
            {
                msg = line_barcode_pack(sensors, 6u, 0u, LINE_BARCODE_FAULT,
                    LINE_BARCODE_EVT_FAULT, LINE_BARCODE_ERR_LINE);
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
