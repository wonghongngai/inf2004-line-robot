/* Buddy 1: task-safe binary communication API. */
#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H
#include "../app.h"
#include "communication_protocol.h"
/* Pack a task-local snapshot, then queue it. E_OK means queued, not delivered. */
IMPORT ER subsystem1_publish_telemetry(const S1_TELEMETRY *packet);
/* Returns 1 on success, negative kernel error otherwise. No command execution. */
IMPORT INT subsystem1_receive_command(S1_COMMAND *command, TMO timeout);
/* Validated barcode map from MQTT; returns S1_MAPPING_BYTES on success.
 * The controller applies it with S3's line_service_set_mapping(). */
IMPORT INT subsystem1_receive_mapping(S1_MAPPING *mapping, TMO timeout);
#endif
