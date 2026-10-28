/** @file line_demo.c
 * @brief Bench demo task: configure the IR service and print its events.
 *
 * Flow printed on the USB console:
 *   BARCODE 'A' stored -> ... -> JUNCTION -> action LEFT (letter A)
 * No motors are driven; the action line is what Buddy 2 would execute.
 */
#include "line_demo.h"
#include "line_service.h"

#define DEMO_STATUS_MS      1000    /* sensor print period when idle */
#define DEMO_FAULT_PAUSE_MS 1000    /* wait before clearing a fault */

static char const * const g_cmd_names[4] =
{
    "LEFT", "RIGHT", "STRAIGHT", "U-TURN"
};

static char const * const g_state_names[8] =
{
    "FOLLOW", "SEARCH", "JUNCTION", "SCANNING", "PAUSED", "FAULT", "?", "?"
};

static char const * const g_position_names[8] =
{
    "far-left", "left", "centre", "right", "far-right", "junction",
    "none", "?"
};

static void print_faults(uint8_t faults)
{
    tm_printf((UB *)"[S3] FAULT:%s%s%s\n",
              (0u != (faults & LINE_BARCODE_ERR_LINE))
              ? " line-lost-too-long" : "",
              (0u != (faults & LINE_BARCODE_ERR_QUEUE))
              ? " event-queue-full" : "",
              (0u != (faults & LINE_BARCODE_ERR_TIMING))
              ? " sampling-late" : "");
}

/* Mirror of the service's stored letter, for the printout only. */
static char g_pending_letter = '\0';

static void print_status(line_barcode_msg_t const *p_msg)
{
    uint8_t sensors = p_msg->data[0] & 7u;
    tm_printf((UB *)"[S3] sensors L=%d C=%d R=%d (1=black) pos=%s state=%s"
              " waiting=%c\n",
              (sensors >> 2u) & 1u, (sensors >> 1u) & 1u, sensors & 1u,
              g_position_names[(p_msg->data[0] >> 3u) & 7u],
              g_state_names[p_msg->data[1] & 7u],
              ('\0' != g_pending_letter) ? g_pending_letter : '-');
}

static void print_event(line_barcode_msg_t const *p_msg)
{
    uint8_t event = (p_msg->data[1] >> 3u) & 7u;
    char letter = (char)('A' + (p_msg->data[2] & LINE_BARCODE_LETTER_MASK));

    switch (event)
    {
        case LINE_BARCODE_EVT_BARCODE:
            g_pending_letter = letter;
            tm_printf((UB *)"[S3] BARCODE '%c' stored, waiting for junction\n",
                      letter);
        break;

        case LINE_BARCODE_EVT_COMMAND:
            g_pending_letter = '\0';
            tm_printf((UB *)"[S3] JUNCTION -> action %s (letter %c)\n",
                      g_cmd_names[p_msg->data[0] >> 6u], letter);
        break;

        case LINE_BARCODE_EVT_JUNCTION:
            if ('\0' != g_pending_letter)
            {
                tm_printf((UB *)"[S3] JUNCTION -> letter %c has no mapping,"
                          " no action\n", g_pending_letter);
            }
            else
            {
                tm_printf((UB *)"[S3] JUNCTION, no stored letter\n");
            }
            g_pending_letter = '\0';
        break;

        case LINE_BARCODE_EVT_INVALID:
            tm_printf((UB *)"[S3] barcode unreadable, ignored\n");
        break;

        case LINE_BARCODE_EVT_LOST:
            tm_printf((UB *)"[S3] line LOST, searching\n");
        break;

        case LINE_BARCODE_EVT_FOUND:
            tm_printf((UB *)"[S3] line FOUND\n");
        break;

        case LINE_BARCODE_EVT_FAULT:
            print_faults(p_msg->data[2] >> LINE_BARCODE_FAULT_SHIFT);
        break;

        default:
            /* STATUS is never queued. */
        break;
    }
}

static ER demo_configure(void)
{
    ER result = line_service_calibrate(LINE_DEMO_BLACK_RAW,
                                       (uint8_t)(~LINE_DEMO_BLACK_RAW & 7u));
    if (E_OK == result)
    {
        result = line_service_set_layout(LINE_BARCODE_LAYOUT_SIDE);
    }
    if (E_OK == result)
    {
        result = line_service_set_mapping(LINE_DEMO_MAP_LEFT,
                                          LINE_DEMO_MAP_RIGHT,
                                          LINE_DEMO_MAP_STRAIGHT,
                                          LINE_DEMO_MAP_UTURN);
    }
    if (E_OK == result)
    {
        result = line_service_enable(true);
    }
    return (result);
}

static void line_demo_task(INT start_code, void *p_extra)
{
    line_barcode_msg_t msg = {{0u, 0u, 0u}};
    (void)start_code;
    (void)p_extra;

    ER result = demo_configure();
    CHECK(result, "line_demo_configure");
    tm_printf((UB *)"[S3] demo map: %c=LEFT %c=RIGHT %c=STRAIGHT %c=U-TURN\n",
              LINE_DEMO_MAP_LEFT, LINE_DEMO_MAP_RIGHT,
              LINE_DEMO_MAP_STRAIGHT, LINE_DEMO_MAP_UTURN);

    for (;;)
    {
        result = line_service_receive(&msg, DEMO_STATUS_MS);
        if (E_OK == result)
        {
            print_event(&msg);
        }
        else if (E_TMOUT == result)
        {
            result = line_service_snapshot(&msg);
            if (E_OK == result)
            {
                print_status(&msg);
            }
            else
            {
                tm_printf((UB *)"[S3] no fresh IR sample (%d)\n", result);
            }
        }
        else
        {
            CHECK(result, "line_service_receive");
        }

        /* Bench convenience: a latched fault (e.g. robot lifted for 2 s)
         * is cleared so testing can continue. The real controller must
         * stop the motors instead.
         */
        if ((E_OK == line_service_snapshot(&msg))
            && (LINE_BARCODE_FAULT == (msg.data[1] & 7u)))
        {
            (void)tk_dly_tsk(DEMO_FAULT_PAUSE_MS);
            tm_printf((UB *)"[S3] clearing fault, resuming\n");
            CHECK(line_service_enable(true), "line_service_enable");
        }
    }
}

ER line_demo_start(void)
{
    ID task_id = app_start_task(line_demo_task, LINE_DEMO_PRIORITY,
                                TP_PRC1, "line_demo_task");
    return ((task_id < E_OK) ? task_id : E_OK);
}

/*** end of file ***/
