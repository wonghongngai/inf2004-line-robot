/* Buddy 1 owns bounded RTOS queues; the built-in Wi-Fi task owns the radio. */
#include "subsystem1.h"
#include "communication_internal.h"

LOCAL ID telemetry_queue;
LOCAL ID command_queue;
LOCAL ID mapping_queue;
/* Barcode results get their own queue so they never compete with telemetry. */
LOCAL ID barcode_queue;

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
    if (telemetry_queue > 0 && command_queue > 0 && mapping_queue > 0 &&
        barcode_queue > 0)
    {
        return E_OK;
    }
    telemetry_queue = create_queue(S1_TELEMETRY_BYTES);
    if (telemetry_queue < E_OK)
    {
        return telemetry_queue;
    }
    command_queue = create_queue(S1_COMMAND_BYTES);
    if (command_queue < E_OK)
    {
        tk_del_mbf(telemetry_queue);
        telemetry_queue = 0;
        return command_queue;
    }
    mapping_queue = create_queue(S1_MAPPING_BYTES);
    if (mapping_queue < E_OK)
    {
        tk_del_mbf(command_queue);
        tk_del_mbf(telemetry_queue);
        command_queue = telemetry_queue = 0;
        return mapping_queue;
    }
    barcode_queue = create_queue((INT)sizeof(S1_BARCODE_RESULT));
    if (barcode_queue < E_OK)
    {
        tk_del_mbf(mapping_queue);
        tk_del_mbf(command_queue);
        tk_del_mbf(telemetry_queue);
        mapping_queue = command_queue = telemetry_queue = 0;
        return barcode_queue;
    }
    return E_OK;
}

EXPORT ER subsystem1_start(void)
{
    /* No second network task: cyw43_utk_task services all queues. */
    return (telemetry_queue > 0 && command_queue > 0 && mapping_queue > 0 &&
            barcode_queue > 0)
               ? E_OK
               : E_OBJ;
}

EXPORT ER subsystem1_publish_telemetry(const S1_TELEMETRY * packet)
{
    if (!s1_telemetry_valid(packet))
    {
        return E_PAR;
    }
#if !TM_MQTT
    return E_NOSPT;
#else
    if (telemetry_queue <= 0)
    {
        return E_OBJ;
    }
    return tk_snd_mbf(telemetry_queue, (void *)packet->bytes,
                      S1_TELEMETRY_BYTES, TMO_POL);
#endif
}

EXPORT ER subsystem1_publish_barcode(const S1_BARCODE_RESULT * result)
{
    if (!s1_barcode_result_valid(result))
    {
        return E_PAR;
    }
#if !TM_MQTT
    return E_NOSPT;
#else
    if (barcode_queue <= 0)
    {
        return E_OBJ;
    }
    /* Nonblocking: a full queue returns E_TMOUT and the event is dropped. */
    return tk_snd_mbf(barcode_queue, (void *)result, (INT)sizeof(*result),
                      TMO_POL);
#endif
}

EXPORT INT subsystem1_receive_command(S1_COMMAND * command, TMO timeout)
{
    if (command == NULL)
    {
        return E_PAR;
    }
    if (command_queue <= 0)
    {
        return E_OBJ;
    }
    return tk_rcv_mbf(command_queue, command->bytes, timeout);
}

EXPORT INT subsystem1_receive_mapping(S1_MAPPING * mapping, TMO timeout)
{
    if (mapping == NULL)
    {
        return E_PAR;
    }
    if (mapping_queue <= 0)
    {
        return E_OBJ;
    }
    return tk_rcv_mbf(mapping_queue, mapping->bytes, timeout);
}

int s1_take_telemetry(unsigned char * payload)
{
    if (telemetry_queue <= 0)
    {
        return E_OBJ;
    }
    return tk_rcv_mbf(telemetry_queue, payload, TMO_POL);
}

int s1_take_barcode(S1_BARCODE_RESULT * result)
{
    if (result == NULL)
    {
        return E_PAR;
    }
    if (barcode_queue <= 0)
    {
        return E_OBJ;
    }
    return tk_rcv_mbf(barcode_queue, result, TMO_POL);
}

/* Command messages that print their own lines per report window. */
#define S1_COMMAND_LOG_BURST 4U

LOCAL S1_COMMAND_STATS command_stats;
LOCAL S1_COMMAND_STATS command_stats_reported;
LOCAL unsigned int     command_log_budget = S1_COMMAND_LOG_BURST;

void s1_command_stats_get(S1_COMMAND_STATS * out)
{
    if (out != NULL)
    {
        *out = command_stats;
    }
}

int s1_command_log_ok(void)
{
    if (command_log_budget == 0U)
    {
        return 0;
    }
    --command_log_budget;
    return 1;
}

void s1_command_stats_report(void)
{
    const S1_COMMAND_STATS * now  = &command_stats;
    const S1_COMMAND_STATS * last = &command_stats_reported;
    if (now->received - last->received > S1_COMMAND_LOG_BURST ||
        now->invalid != last->invalid || now->dropped != last->dropped)
    {
        LOG_INFO(
            TAG_S1_CMD,
            "commands rx=%u queued=%u invalid=%u dropped=%u (+%u this window)",
            now->received, now->queued, now->invalid, now->dropped,
            now->received - last->received);
    }
    command_stats_reported = command_stats;
    command_log_budget     = S1_COMMAND_LOG_BURST;
}

void s1_deliver_command(const unsigned char * payload, unsigned int length)
{
    S1_COMMAND command;
    ER         result;
    if (payload == NULL || command_queue <= 0)
    {
        return;
    }
    ++command_stats.received;
    if (length != S1_COMMAND_BYTES)
    {
        ++command_stats.invalid;
        return;
    }
    command.bytes[0] = payload[0];
    if (!s1_command_valid(&command))
    {
        ++command_stats.invalid;
        return;
    }
    /* TMO_POL: a full queue drops the command instead of blocking the
     * Wi-Fi task (S1-U17). Only the first drop of a window prints. */
    result =
        tk_snd_mbf(command_queue, command.bytes, S1_COMMAND_BYTES, TMO_POL);
    if (result < E_OK)
    {
        if (command_stats.dropped == command_stats_reported.dropped)
        {
            LOG_ERR(TAG_S1_CMD,
                    "command queue full/error %d; counting further drops",
                    result);
        }
        ++command_stats.dropped;
    }
    else
    {
        ++command_stats.queued;
    }
}

void s1_deliver_mapping(const unsigned char * payload, unsigned int length)
{
    S1_MAPPING    mapping;
    unsigned char letters[S1_MAP_SLOTS];
    unsigned int  i;
    ER            result;
    if (payload == NULL || length != S1_MAPPING_BYTES || mapping_queue <= 0)
    {
        return;
    }
    for (i = 0; i < S1_MAPPING_BYTES; ++i)
    {
        mapping.bytes[i] = payload[i];
    }
    if (!s1_mapping_get(&mapping, letters))
    {
        LOG_WARN(TAG_S1_CMD, "invalid barcode map rejected");
        return;
    }
    result =
        tk_snd_mbf(mapping_queue, mapping.bytes, S1_MAPPING_BYTES, TMO_POL);
    if (result < E_OK)
    {
        LOG_ERR(TAG_S1_CMD, "mapping queue full/error %d", result);
    }
}
