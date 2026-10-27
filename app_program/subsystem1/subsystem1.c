/* Buddy 1 owns bounded RTOS queues; the built-in Wi-Fi task owns the radio. */
#include "subsystem1.h"
#include "communication_internal.h"

LOCAL ID telemetry_queue;
LOCAL ID command_queue;

LOCAL ID create_queue(INT message_size)
{
    T_CMBF config = {0};
    config.mbfatr = TA_TFIFO;
    /* Kernel message lengths and payloads are aligned to sizeof(INT). */
    config.bufsz = S1_QUEUE_DEPTH * (sizeof(INT) +
                    ((message_size + sizeof(INT) - 1) / sizeof(INT)) * sizeof(INT));
    config.maxmsz = message_size;
    return tk_cre_mbf(&config);
}

EXPORT ER subsystem1_init(void)
{
    if (telemetry_queue > 0 && command_queue > 0) {
        return E_OK;
    }
    telemetry_queue = create_queue(S1_TELEMETRY_BYTES);
    if (telemetry_queue < E_OK) {
        return telemetry_queue;
    }
    command_queue = create_queue(S1_COMMAND_BYTES);
    if (command_queue < E_OK) {
        tk_del_mbf(telemetry_queue);
        telemetry_queue = 0;
        return command_queue;
    }
    return E_OK;
}

EXPORT ER subsystem1_start(void)
{
    /* No second network task: cyw43_utk_task services both queues. */
    return (telemetry_queue > 0 && command_queue > 0) ? E_OK : E_OBJ;
}

EXPORT ER subsystem1_publish_telemetry(const S1_TELEMETRY *packet)
{
    if (!s1_telemetry_valid(packet)) {
        return E_PAR;
    }
#if !TM_MQTT
    return E_NOSPT;
#else
    if (telemetry_queue <= 0) {
        return E_OBJ;
    }
    return tk_snd_mbf(telemetry_queue, (void *)packet->bytes,
                      S1_TELEMETRY_BYTES, TMO_POL);
#endif
}

EXPORT INT subsystem1_receive_command(S1_COMMAND *command, TMO timeout)
{
    if (command == NULL) {
        return E_PAR;
    }
    if (command_queue <= 0) {
        return E_OBJ;
    }
    return tk_rcv_mbf(command_queue, command->bytes, timeout);
}

int s1_take_telemetry(unsigned char *payload)
{
    if (telemetry_queue <= 0) {
        return E_OBJ;
    }
    return tk_rcv_mbf(telemetry_queue, payload, TMO_POL);
}

void s1_deliver_command(const unsigned char *payload, unsigned int length)
{
    S1_COMMAND command;
    ER result;
    if (payload == NULL || length != S1_COMMAND_BYTES || command_queue <= 0) {
        return;
    }
    command.bytes[0] = payload[0];
    if (!s1_command_valid(&command)) {
        return;
    }
    result = tk_snd_mbf(command_queue, command.bytes, S1_COMMAND_BYTES, TMO_POL);
    if (result < E_OK) {
        tm_printf((UB *)"Buddy 1: command queue full/error %d\n", result);
    }
}
