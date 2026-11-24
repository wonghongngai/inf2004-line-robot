#include "communication_protocol.h"

/* Change ranges above, and widths/offsets here together when revising the
 * protocol. The wire layout is explicit, independent of C struct bitfields. */
#define STATE_BITS    3U
#define FAULT_BITS    5U
#define S5_FAULT_BITS 3U /* bits 8-10 (#46); the battery field was removed */
#define SPEED_BITS    9U
#define DISTANCE_BITS 16U
#define LINE_BITS     3U
#define BARCODE_BITS  3U
#define HUMP_BITS     8U
#define OBSTACLE_BITS 9U
#define SCAN_BITS     8U
#define WIDTH_BITS    9U
#define BARCODE_LETTER_BITS 5U
#define SPEED_VALID_BITS    1U
#define BARCODE_COUNT_BITS  16U
#define MAP_RECEIVED_BITS   1U
#define RESERVED_BITS \
    5U /* bits 11-14 (old battery bits) and bit 87: must be zero */
#define TICKS_BITS 16U /* #41: bits 120-135 left, 136-151 right */
_Static_assert(STATE_BITS + FAULT_BITS + S5_FAULT_BITS + SPEED_BITS +
                       DISTANCE_BITS + LINE_BITS + BARCODE_BITS + HUMP_BITS +
                       OBSTACLE_BITS + SCAN_BITS + WIDTH_BITS +
                       BARCODE_LETTER_BITS + SPEED_VALID_BITS +
                       MAP_RECEIVED_BITS + RESERVED_BITS +
                       2U * BARCODE_COUNT_BITS + 2U * TICKS_BITS ==
                   S1_TELEMETRY_BYTES * 8U,
               "Update packet size and protocol layout together");
_Static_assert(sizeof(S1_TELEMETRY) == S1_TELEMETRY_BYTES,
               "Telemetry must have no padding");
_Static_assert(sizeof(S1_COMMAND) == 1, "Command must have no padding");
#define MAP_LETTER_BITS 5U
_Static_assert(S1_MAP_SLOTS * MAP_LETTER_BITS <= S1_MAPPING_BYTES * 8U &&
                   S1_MAP_SLOTS * MAP_LETTER_BITS >
                       (S1_MAPPING_BYTES - 1U) * 8U,
               "Update mapping size and layout together");
_Static_assert(sizeof(S1_MAPPING) == 3, "Mapping must have no padding");
#define BARCODE_FRAME_BITS    16U
#define BARCODE_RUNS_BITS     5U
#define BARCODE_LETTER_OFFSET 16U
#define BARCODE_RUNS_OFFSET   21U
#define BARCODE_USED_BITS     (BARCODE_RUNS_OFFSET + BARCODE_RUNS_BITS)
_Static_assert(BARCODE_FRAME_BITS + BARCODE_LETTER_BITS + BARCODE_RUNS_BITS ==
                       BARCODE_USED_BITS &&
                   BARCODE_USED_BITS <= S1_BARCODE_BYTES * 8U &&
                   BARCODE_USED_BITS > (S1_BARCODE_BYTES - 1U) * 8U,
               "Update barcode packet size and layout together");
_Static_assert(sizeof(S1_BARCODE_PACKET) == S1_BARCODE_BYTES,
               "Barcode packet must have no padding");

/* Fail compilation if a later range edit no longer fits its allocation,
 * or would leave unnecessary bits. Revisit layout and decoder in that case. */
#define CHECK_BITS(name, low, high, bits)                       \
    _Static_assert((high) - (low) < (1U << (bits)) &&           \
                       (high) - (low) >= (1U << ((bits) - 1U)), \
                   name)
CHECK_BITS("state bits", S1_STATE_MIN, S1_STATE_MAX, STATE_BITS);
CHECK_BITS("fault bits", S1_FAULTS_MIN, S1_FAULTS_MAX, FAULT_BITS);
CHECK_BITS("speed bits", S1_SPEED_MIN, S1_SPEED_MAX, SPEED_BITS);
CHECK_BITS("distance bits", S1_DISTANCE_MIN, S1_DISTANCE_MAX, DISTANCE_BITS);
CHECK_BITS("line bits", S1_LINE_MIN, S1_LINE_MAX, LINE_BITS);
CHECK_BITS("barcode bits", S1_BARCODE_MIN, S1_BARCODE_MAX, BARCODE_BITS);
CHECK_BITS("barcode letter bits", 0, S1_BARCODE_LETTER_MAX,
           BARCODE_LETTER_BITS);
CHECK_BITS("barcode count bits", 0, S1_BARCODE_COUNT_MAX, BARCODE_COUNT_BITS);
CHECK_BITS("s5 fault bits", S1_S5_FAULTS_MIN, S1_S5_FAULTS_MAX, S5_FAULT_BITS);
CHECK_BITS("encoder tick bits", 0, S1_TICKS_MASK, TICKS_BITS);
CHECK_BITS("hump bits", S1_HUMP_MIN, S1_HUMP_MAX, HUMP_BITS);
CHECK_BITS("obstacle bits", S1_OBSTACLE_MIN, S1_OBSTACLE_MAX, OBSTACLE_BITS);
CHECK_BITS("angle bits", S1_SCAN_MIN, S1_SCAN_MAX, SCAN_BITS);
CHECK_BITS("width bits", S1_WIDTH_MIN, S1_WIDTH_MAX, WIDTH_BITS);
CHECK_BITS("map letter bits", 0, S1_MAP_LETTER_MAX, MAP_LETTER_BITS);
CHECK_BITS("barcode runs bits", 0, S1_BARCODE_RUNS_MAX, BARCODE_RUNS_BITS);
CHECK_BITS("barcode frame bits", 0, S1_BARCODE_FRAME_MASK, BARCODE_FRAME_BITS);
#undef CHECK_BITS

/* Descriptors are immutable protocol metadata, not application measurements. */
typedef struct
{
    unsigned char offset;
    unsigned char width;
    int           minimum;
    int           maximum;
} FIELD_LAYOUT;
static const FIELD_LAYOUT fields[S1_FIELD_COUNT] = {
    {0, STATE_BITS, S1_STATE_MIN, S1_STATE_MAX},
    {3, FAULT_BITS, S1_FAULTS_MIN, S1_FAULTS_MAX},
    {8, S5_FAULT_BITS, S1_S5_FAULTS_MIN, S1_S5_FAULTS_MAX},
    {15, SPEED_BITS, S1_SPEED_MIN, S1_SPEED_MAX},
    {24, DISTANCE_BITS, S1_DISTANCE_MIN, S1_DISTANCE_MAX},
    {40, LINE_BITS, S1_LINE_MIN, S1_LINE_MAX},
    {43, BARCODE_BITS, S1_BARCODE_MIN, S1_BARCODE_MAX},
    {46, HUMP_BITS, S1_HUMP_MIN, S1_HUMP_MAX},
    {54, OBSTACLE_BITS, S1_OBSTACLE_MIN, S1_OBSTACLE_MAX},
    {63, SCAN_BITS, S1_SCAN_MIN, S1_SCAN_MAX},
    {71, WIDTH_BITS, S1_WIDTH_MIN, S1_WIDTH_MAX},
    {80, BARCODE_LETTER_BITS, 0, S1_BARCODE_LETTER_MAX},
    {85, SPEED_VALID_BITS, 0, 1},
    {88, BARCODE_COUNT_BITS, 0, S1_BARCODE_COUNT_MAX},
    {104, BARCODE_COUNT_BITS, 0, S1_BARCODE_COUNT_MAX},
    {86, MAP_RECEIVED_BITS, 0, 1},
    {120, TICKS_BITS, 0, (int)S1_TICKS_MASK},
    {136, TICKS_BITS, 0, (int)S1_TICKS_MASK}};

int s1_telemetry_set(S1_TELEMETRY * packet, S1_FIELD field, int value)
{
    unsigned int         i, encoded;
    const FIELD_LAYOUT * layout;
    if (packet == 0 || (unsigned int)field >= S1_FIELD_COUNT)
    {
        return 0;
    }
    layout = &fields[field];
    if (value < layout->minimum || value > layout->maximum)
    {
        return 0;
    }
    encoded = (unsigned int)(value - layout->minimum);
    for (i = 0; i < layout->width; ++i)
    {
        unsigned int    bit  = layout->offset + i;
        unsigned char   mask = (unsigned char)(1U << (bit % 8U));
        unsigned char * byte = &packet->bytes[bit / 8U];
        *byte = (unsigned char)((*byte & (unsigned char)~mask) |
                                (((encoded >> i) & 1U) ? mask : 0U));
    }
    return 1;
}

int s1_telemetry_get(const S1_TELEMETRY * packet, S1_FIELD field, int * value)
{
    unsigned int         i, encoded = 0;
    const FIELD_LAYOUT * layout;
    if (packet == 0 || value == 0 || (unsigned int)field >= S1_FIELD_COUNT)
    {
        return 0;
    }
    layout = &fields[field];
    for (i = 0; i < layout->width; ++i)
    {
        unsigned int bit = layout->offset + i;
        encoded |= ((unsigned int)(packet->bytes[bit / 8U] >> (bit % 8U)) & 1U)
                   << i;
    }
    if (encoded > (unsigned int)(layout->maximum - layout->minimum))
    {
        return 0;
    }
    *value = (int)encoded + layout->minimum;
    return 1;
}

void s1_telemetry_init(S1_TELEMETRY * packet)
{
    unsigned int i;
    if (packet == 0)
    {
        return;
    }
    for (i = 0; i < S1_TELEMETRY_BYTES; ++i)
    {
        packet->bytes[i] = 0;
    }
    /* Encoded speed zero means -200; initialize logical speed to zero. */
    (void)s1_telemetry_set(packet, S1_FIELD_SPEED_CMS, 0);
}

int s1_telemetry_valid(const S1_TELEMETRY * packet)
{
    unsigned int i;
    int          value;
    /* Reserved, must be zero: bits 11-14 (byte 1, 0x78) and bit 87 (byte 10, 0x80).
       Bit 86 is the map flag (#38); bits 8-10 are the S5 faults (#46). */
    if (packet == 0 || (packet->bytes[1] & 0x78U) != 0U ||
        (packet->bytes[10] & 0x80U) != 0U)
        return 0;
    for (i = 0; i < S1_FIELD_COUNT; ++i)
    {
        if (!s1_telemetry_get(packet, (S1_FIELD)i, &value))
        {
            return 0;
        }
    }
    return 1;
}

int s1_command_set(S1_COMMAND * packet, unsigned int code)
{
    if (packet == 0 || code > S1_COMMAND_MAX)
    {
        return 0;
    }
    packet->bytes[0] = (unsigned char)code;
    return 1;
}

int s1_command_valid(const S1_COMMAND * packet)
{
    return packet != 0 && packet->bytes[0] <= S1_COMMAND_MAX;
}

int s1_mapping_get(const S1_MAPPING * packet,
                   unsigned char      letters[S1_MAP_SLOTS])
{
    unsigned char decoded[S1_MAP_SLOTS];
    unsigned int  slot, other, bit;
    if (packet == 0 || letters == 0)
    {
        return 0;
    }
    for (bit = S1_MAP_SLOTS * MAP_LETTER_BITS; bit < S1_MAPPING_BYTES * 8U;
         ++bit)
    {
        if (((packet->bytes[bit / 8U] >> (bit % 8U)) & 1U) != 0U)
        {
            return 0;
        }
    }
    for (slot = 0; slot < S1_MAP_SLOTS; ++slot)
    {
        unsigned int i, value = 0;
        for (i = 0; i < MAP_LETTER_BITS; ++i)
        {
            bit = slot * MAP_LETTER_BITS + i;
            value |=
                ((unsigned int)(packet->bytes[bit / 8U] >> (bit % 8U)) & 1U)
                << i;
        }
        if (value > S1_MAP_LETTER_MAX)
        {
            return 0;
        }
        for (other = 0; other < slot; ++other)
        {
            if (decoded[other] == value)
            {
                return 0;
            }
        }
        decoded[slot] = (unsigned char)value;
    }
    for (slot = 0; slot < S1_MAP_SLOTS; ++slot)
    {
        letters[slot] = decoded[slot];
    }
    return 1;
}

int s1_mapping_set(S1_MAPPING *        packet,
                   const unsigned char letters[S1_MAP_SLOTS])
{
    S1_MAPPING    encoded = {{0}};
    unsigned char check[S1_MAP_SLOTS];
    unsigned int  slot, i;
    if (packet == 0 || letters == 0)
    {
        return 0;
    }
    for (slot = 0; slot < S1_MAP_SLOTS; ++slot)
    {
        if (letters[slot] > S1_MAP_LETTER_MAX)
        {
            return 0;
        }
        for (i = 0; i < MAP_LETTER_BITS; ++i)
        {
            unsigned int bit = slot * MAP_LETTER_BITS + i;
            if (((letters[slot] >> i) & 1U) != 0U)
            {
                encoded.bytes[bit / 8U] |= (unsigned char)(1U << (bit % 8U));
            }
        }
    }
    if (!s1_mapping_get(&encoded, check))
    {
        return 0; /* duplicate letters */
    }
    *packet = encoded;
    return 1;
}

int s1_barcode_result_valid(const S1_BARCODE_RESULT * result)
{
    if (result == 0 || result->runs > S1_BARCODE_RUNS_MAX)
    {
        return 0;
    }
    return result->letter == 0U ||
           (result->letter >= 'A' && result->letter <= 'Z');
}

/* All 32 bits as one little-endian value: bit 0 is the LSB of byte 0. */
int s1_barcode_pack(const S1_BARCODE_RESULT * result,
                    S1_BARCODE_PACKET *       packet)
{
    unsigned long word;
    unsigned int  i;
    if (packet == 0 || !s1_barcode_result_valid(result))
    {
        return 0;
    }
    word = (unsigned long)(result->id & S1_BARCODE_FRAME_MASK);
    if (result->letter != 0U)
    {
        word |= (unsigned long)(result->letter - 'A' + 1U)
                << BARCODE_LETTER_OFFSET;
    }
    word |= (unsigned long)result->runs << BARCODE_RUNS_OFFSET;
    for (i = 0; i < S1_BARCODE_BYTES; ++i)
    {
        packet->bytes[i] = (unsigned char)((word >> (8U * i)) & 0xFFU);
    }
    return 1;
}

int s1_barcode_unpack(const S1_BARCODE_PACKET * packet,
                      S1_BARCODE_RESULT *       result)
{
    unsigned long word = 0;
    unsigned int  i, letter, runs;
    if (packet == 0 || result == 0)
    {
        return 0;
    }
    for (i = 0; i < S1_BARCODE_BYTES; ++i)
    {
        word |= (unsigned long)packet->bytes[i] << (8U * i);
    }
    letter = (unsigned int)((word >> BARCODE_LETTER_OFFSET) &
                            ((1U << BARCODE_LETTER_BITS) - 1U));
    runs   = (unsigned int)((word >> BARCODE_RUNS_OFFSET) &
                            ((1U << BARCODE_RUNS_BITS) - 1U));
    if ((word >> BARCODE_USED_BITS) != 0UL || letter > S1_BARCODE_LETTER_MAX ||
        runs > S1_BARCODE_RUNS_MAX)
    {
        return 0;
    }
    result->id     = (unsigned int)(word & S1_BARCODE_FRAME_MASK);
    result->letter = letter == 0U ? 0U : (unsigned char)('A' + letter - 1U);
    result->runs   = (unsigned char)runs;
    return 1;
}
