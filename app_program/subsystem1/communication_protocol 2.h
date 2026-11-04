/* Binary protocol v1. No SDK/RTOS dependencies; all persistent data is packed. */
#ifndef COMMUNICATION_PROTOCOL_H
#define COMMUNICATION_PROTOCOL_H

#define S1_TELEMETRY_BYTES 10U
#define S1_COMMAND_BYTES 1U
#define S1_HEARTBEAT_BYTES 1U
#define S1_QUEUE_DEPTH 4U

/* Logical field identifiers, never transmitted as enums or C structs. */
typedef enum {
    S1_FIELD_STATE, S1_FIELD_FAULTS, S1_FIELD_BATTERY_PERCENT,
    S1_FIELD_SPEED_CMS, S1_FIELD_DISTANCE_CM, S1_FIELD_LINE_SENSORS,
    S1_FIELD_BARCODE, S1_FIELD_HIGHEST_HUMP_MM, S1_FIELD_OBSTACLE_CM,
    S1_FIELD_SCAN_DEG, S1_FIELD_OBSTACLE_WIDTH_CM, S1_FIELD_COUNT
} S1_FIELD;

#define S1_STATE_MIN 0
#define S1_STATE_MAX 7
#define S1_FAULTS_MIN 0
#define S1_FAULTS_MAX 31
#define S1_BATTERY_MIN 0
#define S1_BATTERY_MAX 100
#define S1_SPEED_MIN (-200)
#define S1_SPEED_MAX 200
#define S1_DISTANCE_MIN 0
#define S1_DISTANCE_MAX 65535
#define S1_LINE_MIN 0
#define S1_LINE_MAX 7
#define S1_BARCODE_MIN 0
#define S1_BARCODE_MAX 4
#define S1_HUMP_MIN 0
#define S1_HUMP_MAX 255
#define S1_OBSTACLE_MIN 0
#define S1_OBSTACLE_MAX 400
#define S1_SCAN_MIN 0
#define S1_SCAN_MAX 180
#define S1_WIDTH_MIN 0
#define S1_WIDTH_MAX 400

#define S1_STATE_IDLE 0
#define S1_STATE_FOLLOWING 1
#define S1_STATE_TURNING 2
#define S1_STATE_SCANNING 3
#define S1_STATE_AVOIDING 4
#define S1_STATE_RECOVERING 5
#define S1_STATE_FINISHED 6
#define S1_STATE_FAULT 7
#define S1_FAULT_MOTOR_STALL (1U << 0)
#define S1_FAULT_ENCODER (1U << 1)
#define S1_FAULT_IMU (1U << 2)
#define S1_FAULT_NO_ECHO (1U << 3)
#define S1_FAULT_WIFI_LOSS (1U << 4)
#define S1_BARCODE_NONE 0
#define S1_BARCODE_LEFT 1
#define S1_BARCODE_RIGHT 2
#define S1_BARCODE_STRAIGHT 3
#define S1_BARCODE_UTURN 4

/* Only two bits carry information; upper six bits must be zero. */
#define S1_COMMAND_STATUS 0U
#define S1_COMMAND_PAUSE 1U
#define S1_COMMAND_RESUME 2U
#define S1_COMMAND_RESET_STATS 3U
#define S1_COMMAND_MASK 3U
#define S1_HEARTBEAT_OFFLINE 0U
#define S1_HEARTBEAT_ONLINE 1U

typedef struct { unsigned char bytes[S1_TELEMETRY_BYTES]; } S1_TELEMETRY;
typedef struct { unsigned char bytes[S1_COMMAND_BYTES]; } S1_COMMAND;

/* Return 1 on success, 0 on invalid input. Failed setters leave packet intact.
 * Values are logical units, including signed speed. Pack before enqueueing;
 * don't modify a packet concurrently from multiple tasks/ISRs. */
void s1_telemetry_init(S1_TELEMETRY *packet);
int s1_telemetry_set(S1_TELEMETRY *packet, S1_FIELD field, int value);
int s1_telemetry_get(const S1_TELEMETRY *packet, S1_FIELD field, int *value);
int s1_telemetry_valid(const S1_TELEMETRY *packet);
int s1_command_set(S1_COMMAND *packet, unsigned int code);
int s1_command_valid(const S1_COMMAND *packet);
#endif
