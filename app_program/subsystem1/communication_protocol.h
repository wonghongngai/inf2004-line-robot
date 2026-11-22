/* Binary protocol v1. No SDK/RTOS dependencies; all persistent data is packed. */
#ifndef COMMUNICATION_PROTOCOL_H
#define COMMUNICATION_PROTOCOL_H

#define S1_TELEMETRY_BYTES 15U
#define S1_COMMAND_BYTES 1U
#define S1_MAPPING_BYTES 3U
#define S1_HEARTBEAT_BYTES 1U
#define S1_QUEUE_DEPTH 4U

/* Logical field identifiers, never transmitted as enums or C structs. */
typedef enum {
    S1_FIELD_STATE, S1_FIELD_FAULTS, S1_FIELD_BATTERY_PERCENT,
    S1_FIELD_SPEED_CMS, S1_FIELD_DISTANCE_CM, S1_FIELD_LINE_SENSORS,
    S1_FIELD_BARCODE, S1_FIELD_HIGHEST_HUMP_MM, S1_FIELD_OBSTACLE_CM,
    S1_FIELD_SCAN_DEG, S1_FIELD_OBSTACLE_WIDTH_CM, S1_FIELD_BARCODE_LETTER,
    S1_FIELD_SPEED_VALID, S1_FIELD_BARCODE_BLACK, S1_FIELD_BARCODE_WHITE,
    S1_FIELD_MAP_RECEIVED, /* #38: 1 once a letter map has been applied */
    S1_FIELD_S5_FAULTS,    /* #46: S5 safe-stop causes, 3 bits (S1_S5_FAULT_*) */
    S1_FIELD_COUNT
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
/* 0 = no decode yet, 1..26 = A..Z. */
#define S1_BARCODE_LETTER_MAX 26
#define S1_BARCODE_COUNT_MAX 32767U /* 15 bits since #46 */
#define S1_S5_FAULTS_MIN 0
#define S1_S5_FAULTS_MAX 7
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
/* #46: bits of S1_FIELD_S5_FAULTS (same values as S5_FAULT_* in subsystem5.h) */
#define S1_S5_FAULT_CAL_FAIL (1U << 0)
#define S1_S5_FAULT_NO_PATH (1U << 1)
#define S1_S5_FAULT_LINE_LOST (1U << 2)
#define S1_BARCODE_NONE 0
#define S1_BARCODE_LEFT 1
#define S1_BARCODE_RIGHT 2
#define S1_BARCODE_STRAIGHT 3
#define S1_BARCODE_UTURN 4

/* Commands use three bits; codes 6–7 and the upper five bits are reserved. */
#define S1_COMMAND_STATUS 0U
#define S1_COMMAND_PAUSE 1U
#define S1_COMMAND_RESUME 2U
#define S1_COMMAND_RESET_STATS 3U
#define S1_COMMAND_REVERSE 4U
/* Leave Ready and start navigating (#48). The integrated robot waits for it. */
#define S1_COMMAND_START 5U
#define S1_COMMAND_MAX S1_COMMAND_START
#define S1_HEARTBEAT_OFFLINE 0U
#define S1_HEARTBEAT_ONLINE 1U

/* Barcode route map: four 5-bit letters (A=0..Z=25) in bits 0-19, slot order
 * left, right, straight, U-turn (S3's command order). Bits 20-23 MUST be zero;
 * the four letters must differ. */
#define S1_MAP_SLOTS 4U
#define S1_MAP_LEFT 0U
#define S1_MAP_RIGHT 1U
#define S1_MAP_STRAIGHT 2U
#define S1_MAP_UTURN 3U
#define S1_MAP_LETTER_MAX 25U

typedef struct { unsigned char bytes[S1_TELEMETRY_BYTES]; } S1_TELEMETRY;
typedef struct { unsigned char bytes[S1_COMMAND_BYTES]; } S1_COMMAND;
typedef struct { unsigned char bytes[S1_MAPPING_BYTES]; } S1_MAPPING;

/* Barcode result event: one per completed or rejected capture. The struct
 * only crosses an RTOS queue on the Pico. On <id>/barcode it is packed into
 * S1_BARCODE_BYTES: frame bits 0-15 (low 16 bits, wraps), letter bits 16-20
 * (0 invalid, 1..26 = A..Z), runs bits 21-25 (0..29); bits 26-31 MUST be zero. */
#define S1_BARCODE_BYTES 4U
#define S1_BARCODE_RUNS_MAX 29U
#define S1_BARCODE_FRAME_MASK 0xFFFFU
typedef struct {
    unsigned int id;      /* per-boot capture frame number */
    unsigned char runs;   /* bars + gaps captured, 0..S1_BARCODE_RUNS_MAX */
    unsigned char letter; /* 'A'..'Z' when decoded, 0 for an invalid capture */
} S1_BARCODE_RESULT;
typedef struct { unsigned char bytes[S1_BARCODE_BYTES]; } S1_BARCODE_PACKET;

/* Return 1 on success, 0 on invalid input. Failed setters leave packet intact.
 * Values are logical units, including signed speed. Pack before enqueueing;
 * don't modify a packet concurrently from multiple tasks/ISRs. */
void s1_telemetry_init(S1_TELEMETRY *packet);
int s1_telemetry_set(S1_TELEMETRY *packet, S1_FIELD field, int value);
int s1_telemetry_get(const S1_TELEMETRY *packet, S1_FIELD field, int *value);
int s1_telemetry_valid(const S1_TELEMETRY *packet);
int s1_command_set(S1_COMMAND *packet, unsigned int code);
int s1_command_valid(const S1_COMMAND *packet);
/* letters[slot] are 0..25. Set rejects range/duplicates and leaves packet
 * intact; get rejects reserved bits, range and duplicates. */
int s1_mapping_set(S1_MAPPING *packet, const unsigned char letters[S1_MAP_SLOTS]);
int s1_mapping_get(const S1_MAPPING *packet, unsigned char letters[S1_MAP_SLOTS]);
/* 1 when non-null, runs <= S1_BARCODE_RUNS_MAX and letter is 0 or 'A'..'Z'. */
int s1_barcode_result_valid(const S1_BARCODE_RESULT *result);
/* Pack keeps only the low 16 bits of id; rejects an invalid result and
 * leaves packet intact. Unpack rejects reserved bits and out-of-range fields. */
int s1_barcode_pack(const S1_BARCODE_RESULT *result, S1_BARCODE_PACKET *packet);
int s1_barcode_unpack(const S1_BARCODE_PACKET *packet, S1_BARCODE_RESULT *result);
#endif
