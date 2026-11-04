#include "communication_protocol.h"

/* Change ranges above, and widths/offsets here together when revising the
 * protocol. The wire layout is explicit, independent of C struct bitfields. */
#define STATE_BITS 3U
#define FAULT_BITS 5U
#define BATTERY_BITS 7U
#define SPEED_BITS 9U
#define DISTANCE_BITS 16U
#define LINE_BITS 3U
#define BARCODE_BITS 3U
#define HUMP_BITS 8U
#define OBSTACLE_BITS 9U
#define SCAN_BITS 8U
#define WIDTH_BITS 9U
_Static_assert(STATE_BITS + FAULT_BITS + BATTERY_BITS + SPEED_BITS +
               DISTANCE_BITS + LINE_BITS + BARCODE_BITS + HUMP_BITS +
               OBSTACLE_BITS + SCAN_BITS + WIDTH_BITS == S1_TELEMETRY_BYTES * 8U,
               "Update packet size and protocol layout together");
_Static_assert(sizeof(S1_TELEMETRY) == 10, "Telemetry must have no padding");
_Static_assert(sizeof(S1_COMMAND) == 1, "Command must have no padding");

/* Fail compilation if a later range edit no longer fits its allocation,
 * or would leave unnecessary bits. Revisit layout and decoder in that case. */
#define CHECK_BITS(name, low, high, bits) \
    _Static_assert((high) - (low) < (1U << (bits)) && \
                   (high) - (low) >= (1U << ((bits) - 1U)), name)
CHECK_BITS("state bits", S1_STATE_MIN, S1_STATE_MAX, STATE_BITS);
CHECK_BITS("fault bits", S1_FAULTS_MIN, S1_FAULTS_MAX, FAULT_BITS);
CHECK_BITS("battery bits", S1_BATTERY_MIN, S1_BATTERY_MAX, BATTERY_BITS);
CHECK_BITS("speed bits", S1_SPEED_MIN, S1_SPEED_MAX, SPEED_BITS);
CHECK_BITS("distance bits", S1_DISTANCE_MIN, S1_DISTANCE_MAX, DISTANCE_BITS);
CHECK_BITS("line bits", S1_LINE_MIN, S1_LINE_MAX, LINE_BITS);
CHECK_BITS("barcode bits", S1_BARCODE_MIN, S1_BARCODE_MAX, BARCODE_BITS);
CHECK_BITS("hump bits", S1_HUMP_MIN, S1_HUMP_MAX, HUMP_BITS);
CHECK_BITS("obstacle bits", S1_OBSTACLE_MIN, S1_OBSTACLE_MAX, OBSTACLE_BITS);
CHECK_BITS("angle bits", S1_SCAN_MIN, S1_SCAN_MAX, SCAN_BITS);
CHECK_BITS("width bits", S1_WIDTH_MIN, S1_WIDTH_MAX, WIDTH_BITS);
#undef CHECK_BITS

/* Descriptors are immutable protocol metadata, not application measurements. */
typedef struct {
    unsigned char offset;
    unsigned char width;
    int minimum;
    int maximum;
} FIELD_LAYOUT;
static const FIELD_LAYOUT fields[S1_FIELD_COUNT] = {
    {0, STATE_BITS, S1_STATE_MIN, S1_STATE_MAX},
    {3, FAULT_BITS, S1_FAULTS_MIN, S1_FAULTS_MAX},
    {8, BATTERY_BITS, S1_BATTERY_MIN, S1_BATTERY_MAX},
    {15, SPEED_BITS, S1_SPEED_MIN, S1_SPEED_MAX},
    {24, DISTANCE_BITS, S1_DISTANCE_MIN, S1_DISTANCE_MAX},
    {40, LINE_BITS, S1_LINE_MIN, S1_LINE_MAX},
    {43, BARCODE_BITS, S1_BARCODE_MIN, S1_BARCODE_MAX},
    {46, HUMP_BITS, S1_HUMP_MIN, S1_HUMP_MAX},
    {54, OBSTACLE_BITS, S1_OBSTACLE_MIN, S1_OBSTACLE_MAX},
    {63, SCAN_BITS, S1_SCAN_MIN, S1_SCAN_MAX},
    {71, WIDTH_BITS, S1_WIDTH_MIN, S1_WIDTH_MAX}
};

int s1_telemetry_set(S1_TELEMETRY *packet, S1_FIELD field, int value)
{
    unsigned int i, encoded;
    const FIELD_LAYOUT *layout;
    if (packet == 0 || (unsigned int)field >= S1_FIELD_COUNT) {
        return 0;
    }
    layout = &fields[field];
    if (value < layout->minimum || value > layout->maximum) {
        return 0;
    }
    encoded = (unsigned int)(value - layout->minimum);
    for (i = 0; i < layout->width; ++i) {
        unsigned int bit = layout->offset + i;
        unsigned char mask = (unsigned char)(1U << (bit % 8U));
        unsigned char *byte = &packet->bytes[bit / 8U];
        *byte = (unsigned char)((*byte & (unsigned char)~mask) |
                               (((encoded >> i) & 1U) ? mask : 0U));
    }
    return 1;
}

int s1_telemetry_get(const S1_TELEMETRY *packet, S1_FIELD field, int *value)
{
    unsigned int i, encoded = 0;
    const FIELD_LAYOUT *layout;
    if (packet == 0 || value == 0 || (unsigned int)field >= S1_FIELD_COUNT) {
        return 0;
    }
    layout = &fields[field];
    for (i = 0; i < layout->width; ++i) {
        unsigned int bit = layout->offset + i;
        encoded |= ((unsigned int)(packet->bytes[bit / 8U] >> (bit % 8U)) & 1U) << i;
    }
    if (encoded > (unsigned int)(layout->maximum - layout->minimum)) {
        return 0;
    }
    *value = (int)encoded + layout->minimum;
    return 1;
}

void s1_telemetry_init(S1_TELEMETRY *packet)
{
    unsigned int i;
    if (packet == 0) {
        return;
    }
    for (i = 0; i < S1_TELEMETRY_BYTES; ++i) {
        packet->bytes[i] = 0;
    }
    /* Encoded speed zero means -200; initialize logical speed to zero. */
    (void)s1_telemetry_set(packet, S1_FIELD_SPEED_CMS, 0);
}

int s1_telemetry_valid(const S1_TELEMETRY *packet)
{
    unsigned int i;
    int value;
    for (i = 0; i < S1_FIELD_COUNT; ++i) {
        if (!s1_telemetry_get(packet, (S1_FIELD)i, &value)) {
            return 0;
        }
    }
    return 1;
}

int s1_command_set(S1_COMMAND *packet, unsigned int code)
{
    if (packet == 0 || code > S1_COMMAND_MASK) {
        return 0;
    }
    packet->bytes[0] = (unsigned char)code;
    return 1;
}

int s1_command_valid(const S1_COMMAND *packet)
{
    return packet != 0 && (packet->bytes[0] & ~S1_COMMAND_MASK) == 0U;
}
