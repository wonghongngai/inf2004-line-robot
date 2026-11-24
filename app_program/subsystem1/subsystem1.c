/* Buddy 1 owns bounded RTOS queues; the built-in Wi-Fi task owns the radio. */
#include "subsystem1.h"
#include "communication_internal.h"

LOCAL ID g_telemetry_queue;
LOCAL ID g_command_queue;
LOCAL ID g_mapping_queue;
/* Barcode results get their own queue so they never compete with telemetry. */
LOCAL ID g_barcode_queue;

LOCAL ID create_queue(INT message_size)
{
    T_CMBF config = {0};
    config.mbfatr = TA_TFIFO;
    /* Kernel message lengths and payloads are aligned to sizeof(INT). */
    config.bufsz =
        S1_QUEUE_DEPTH *
        (sizeof(INT) +
         ((message_size + sizeof(INT) - 1) / sizeof(INT)) * sizeof(INT));
    config.maxmsz = message_size;
    return tk_cre_mbf(&config);
}

EXPORT ER subsystem1_init(void)
{
    if (g_telemetry_queue > 0 && g_command_queue > 0 && g_mapping_queue > 0 &&
        g_barcode_queue > 0)
    {
        return E_OK;
    }
    g_telemetry_queue = create_queue(S1_TELEMETRY_BYTES);
    if (g_telemetry_queue < E_OK)
    {
        return g_telemetry_queue;
    }
    g_command_queue = create_queue(S1_COMMAND_BYTES);
    if (g_command_queue < E_OK)
    {
        tk_del_mbf(g_telemetry_queue);
        g_telemetry_queue = 0;
        return g_command_queue;
    }
    g_mapping_queue = create_queue(S1_MAPPING_BYTES);
    if (g_mapping_queue < E_OK)
    {
        tk_del_mbf(g_command_queue);
        tk_del_mbf(g_telemetry_queue);
        g_command_queue = g_telemetry_queue = 0;
        return g_mapping_queue;
    }
    g_barcode_queue = create_queue((INT)sizeof(S1_BARCODE_RESULT));
    if (g_barcode_queue < E_OK)
    {
        tk_del_mbf(g_mapping_queue);
        tk_del_mbf(g_command_queue);
        tk_del_mbf(g_telemetry_queue);
        g_mapping_queue = g_command_queue = g_telemetry_queue = 0;
        return g_barcode_queue;
    }
    return E_OK;
}

EXPORT ER subsystem1_start(void)
{
    /* No second network task: cyw43_utk_task services all queues. */
    return (g_telemetry_queue > 0 && g_command_queue > 0 &&
            g_mapping_queue > 0 && g_barcode_queue > 0)
               ? E_OK
               : E_OBJ;
}

EXPORT ER subsystem1_publish_telemetry(const S1_TELEMETRY * p_packet)
{
    if (!s1_telemetry_valid(p_packet))
    {
        return E_PAR;
    }
#if !TM_MQTT
    return E_NOSPT;
#else
    if (g_telemetry_queue <= 0)
    {
        return E_OBJ;
    }
    return tk_snd_mbf(g_telemetry_queue, (void *)p_packet->bytes,
                      S1_TELEMETRY_BYTES, TMO_POL);
#endif
}

EXPORT ER subsystem1_publish_barcode(const S1_BARCODE_RESULT * p_result)
{
    if (!s1_barcode_result_valid(p_result))
    {
        return E_PAR;
    }
#if !TM_MQTT
    return E_NOSPT;
#else
    if (g_barcode_queue <= 0)
    {
        return E_OBJ;
    }
    /* Nonblocking: a full queue returns E_TMOUT and the event is dropped. */
    return tk_snd_mbf(g_barcode_queue, (void *)p_result, (INT)sizeof(*p_result),
                      TMO_POL);
#endif
}

EXPORT INT subsystem1_receive_command(S1_COMMAND * p_command, TMO timeout)
{
    if (p_command == NULL)
    {
        return E_PAR;
    }
    if (g_command_queue <= 0)
    {
        return E_OBJ;
    }
    return tk_rcv_mbf(g_command_queue, p_command->bytes, timeout);
}

EXPORT INT subsystem1_receive_mapping(S1_MAPPING * p_mapping, TMO timeout)
{
    if (p_mapping == NULL)
    {
        return E_PAR;
    }
    if (g_mapping_queue <= 0)
    {
        return E_OBJ;
    }
    return tk_rcv_mbf(g_mapping_queue, p_mapping->bytes, timeout);
}

int s1_take_telemetry(unsigned char * p_payload)
{
    if (g_telemetry_queue <= 0)
    {
        return E_OBJ;
    }
    return tk_rcv_mbf(g_telemetry_queue, p_payload, TMO_POL);
}

int s1_take_barcode(S1_BARCODE_RESULT * p_result)
{
    if (p_result == NULL)
    {
        return E_PAR;
    }
    if (g_barcode_queue <= 0)
    {
        return E_OBJ;
    }
    return tk_rcv_mbf(g_barcode_queue, p_result, TMO_POL);
}

/* Command messages that print their own lines per report window. */
#define S1_COMMAND_LOG_BURST 4U

LOCAL S1_COMMAND_STATS g_command_stats;
LOCAL S1_COMMAND_STATS g_command_stats_reported;
LOCAL unsigned int     g_command_log_budget = S1_COMMAND_LOG_BURST;

void s1_command_stats_get(S1_COMMAND_STATS * p_out)
{
    if (p_out != NULL)
    {
        *p_out = g_command_stats;
    }
}

int s1_command_log_ok(void)
{
    if (g_command_log_budget == 0U)
    {
        return 0;
    }
    --g_command_log_budget;
    return 1;
}

void s1_command_stats_report(void)
{
    const S1_COMMAND_STATS * p_now  = &g_command_stats;
    const S1_COMMAND_STATS * p_last = &g_command_stats_reported;
    if (p_now->received - p_last->received > S1_COMMAND_LOG_BURST ||
        p_now->invalid != p_last->invalid || p_now->dropped != p_last->dropped)
    {
        LOG_INFO(
            TAG_S1_CMD,
            "commands rx=%u queued=%u invalid=%u dropped=%u (+%u this window)",
            p_now->received, p_now->queued, p_now->invalid, p_now->dropped,
            p_now->received - p_last->received);
    }
    g_command_stats_reported = g_command_stats;
    g_command_log_budget     = S1_COMMAND_LOG_BURST;
}

void s1_deliver_command(const unsigned char * p_payload, unsigned int length)
{
    S1_COMMAND command;
    ER         result;
    if (p_payload == NULL || g_command_queue <= 0)
    {
        return;
    }
    ++g_command_stats.received;
    if (length != S1_COMMAND_BYTES)
    {
        ++g_command_stats.invalid;
        return;
    }
    command.bytes[0] = p_payload[0];
    if (!s1_command_valid(&command))
    {
        ++g_command_stats.invalid;
        return;
    }
    /* TMO_POL: a full queue drops the command instead of blocking the
     * Wi-Fi task (S1-U17). Only the first drop of a window prints. */
    result =
        tk_snd_mbf(g_command_queue, command.bytes, S1_COMMAND_BYTES, TMO_POL);
    if (result < E_OK)
    {
        if (g_command_stats.dropped == g_command_stats_reported.dropped)
        {
            LOG_ERR(TAG_S1_CMD,
                    "command queue full/error %d; counting further drops",
                    result);
        }
        ++g_command_stats.dropped;
    }
    else
    {
        ++g_command_stats.queued;
    }
}

void s1_deliver_mapping(const unsigned char * p_payload, unsigned int length)
{
    S1_MAPPING    mapping;
    unsigned char letters[S1_MAP_SLOTS];
    unsigned int  idx;
    ER            result;
    if (p_payload == NULL || length != S1_MAPPING_BYTES || g_mapping_queue <= 0)
    {
        return;
    }
    for (idx = 0; idx < S1_MAPPING_BYTES; ++idx)
    {
        mapping.bytes[idx] = p_payload[idx];
    }
    if (!s1_mapping_get(&mapping, letters))
    {
        LOG_WARN(TAG_S1_CMD, "invalid barcode map rejected");
        return;
    }
    result =
        tk_snd_mbf(g_mapping_queue, mapping.bytes, S1_MAPPING_BYTES, TMO_POL);
    if (result < E_OK)
    {
        LOG_ERR(TAG_S1_CMD, "mapping queue full/error %d", result);
    }
}
