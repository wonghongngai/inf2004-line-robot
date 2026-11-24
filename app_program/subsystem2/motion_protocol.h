/*
 * motion_protocol.h
 * S2 interface v2: explicit bytes, no compiler enum/padding on the wire.
 *
 * Command: 5 bytes (type + binary32 value), unchanged from v1.
 * Status:  12 bytes, bit-packed little-endian.  The field widths were
 *          decided from robot measurements on 23 Nov 2026 (#52, CR1); the
 *          arithmetic is in testing/subsystem2/TEST_LOG.md (S2-U14).
 *
 * Requires IEEE binary32 (the RP2040 toolchain).  No libm: the kernel build
 * supplies its own headers.
 */
#ifndef S2_MOTION_PROTOCOL_H
#define S2_MOTION_PROTOCOL_H

#include "motion_control.h"
#include <float.h>

_Static_assert(sizeof(float) == 4 && FLT_RADIX == 2 && FLT_MANT_DIG == 24,
               "S2 protocol requires binary32 floats");

/*
 * Status field widths (bits).  These are the single source of truth for
 * every packet that carries S2 numbers (S1 telemetry included, #41).
 *
 * Ticks:    low 16 bits of the cumulative int32 count; WRAPS.  Readers must
 *           take differences with s2_tick_delta().  Highest measured rate
 *           2,080 ticks/s, so two reads may be up to 15.7 s apart.
 * Speed:    signed cm/s, saturating at -128..127 (measured peak 18.4 cm/s;
 *           the speed estimate's own resolution is 0.88 cm/s).
 * Distance: signed cm, saturating at -32768..32767 (longest measured
 *           follow stretch about 54 m = 5,400 cm).
 * PWM:      signed 0.1 % steps, saturating at -1000..1000 (the hardware
 *           PWM resolution is 0.1 %, wrap 1000).
 */
#define S2_STATUS_TICK_BITS     16U
#define S2_STATUS_SPEED_BITS    8U
#define S2_STATUS_DISTANCE_BITS 16U
#define S2_STATUS_PWM_BITS      11U

#define S2_STATUS_SPEED_MIN    (-128)
#define S2_STATUS_SPEED_MAX    127
#define S2_STATUS_DISTANCE_MIN (-32768)
#define S2_STATUS_DISTANCE_MAX 32767
#define S2_STATUS_PWM_SCALE    10.0f /* raw units per percent */
#define S2_STATUS_PWM_RAW_MAX  1000

/* Bit offsets inside S2_PACKED_STATUS (bit 0 = byte 0 bit 0). */
#define S2_STATUS_BIT_STATE      0U /* 3 bits */
#define S2_STATUS_BIT_COMMAND    3U /* 3 bits */
#define S2_STATUS_BIT_FAULT      6U /* 2 bits */
#define S2_STATUS_BIT_COMPLETE   8U /* 1 bit  */
#define S2_STATUS_BIT_LEFT_TICKS 9U
#define S2_STATUS_BIT_RIGHT_TICKS \
    (S2_STATUS_BIT_LEFT_TICKS + S2_STATUS_TICK_BITS)
#define S2_STATUS_BIT_LEFT_SPEED \
    (S2_STATUS_BIT_RIGHT_TICKS + S2_STATUS_TICK_BITS)
#define S2_STATUS_BIT_RIGHT_SPEED \
    (S2_STATUS_BIT_LEFT_SPEED + S2_STATUS_SPEED_BITS)
#define S2_STATUS_BIT_DISTANCE \
    (S2_STATUS_BIT_RIGHT_SPEED + S2_STATUS_SPEED_BITS)
#define S2_STATUS_BIT_LEFT_PWM \
    (S2_STATUS_BIT_DISTANCE + S2_STATUS_DISTANCE_BITS)
#define S2_STATUS_BIT_RIGHT_PWM (S2_STATUS_BIT_LEFT_PWM + S2_STATUS_PWM_BITS)
#define S2_STATUS_BITS_USED \
    (S2_STATUS_BIT_RIGHT_PWM + S2_STATUS_PWM_BITS)        /* 95 */
#define S2_STATUS_BYTES ((S2_STATUS_BITS_USED + 7U) / 8U) /* 12 */

_Static_assert(S2_STATUS_BITS_USED == 95U, "S2 status layout changed");

typedef struct
{
    uint8_t data[5];
} S2_PACKED_COMMAND;
typedef struct
{
    uint8_t data[S2_STATUS_BYTES];
} S2_PACKED_STATUS;

/* ---- byte and bit helpers ---------------------------------------------- */

static inline void s2_put_u32(uint8_t * p, uint32_t v)
{
    for (unsigned i = 0U; i < 4U; ++i)
    {
        p[i] = (uint8_t)(v >> (8U * i));
    }
}

static inline uint32_t s2_get_u32(const uint8_t * p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static inline void s2_put_float(uint8_t * p, float v)
{
    union
    {
        float    f;
        uint32_t u;
    } bits = {.f = v};
    s2_put_u32(p, bits.u);
}

static inline float s2_get_float(const uint8_t * p)
{
    union
    {
        float    f;
        uint32_t u;
    } bits = {.u = s2_get_u32(p)};
    return bits.f;
}

/* Write the low 'width' bits of 'value' at bit 'offset' (little-endian). */
static inline void s2_put_bits(uint8_t * p, unsigned offset, unsigned width,
                               uint32_t value)
{
    for (unsigned i = 0U; i < width; ++i)
    {
        unsigned bit  = offset + i;
        uint8_t  mask = (uint8_t)(1U << (bit % 8U));
        if (((value >> i) & 1U) != 0U)
        {
            p[bit / 8U] |= mask;
        }
        else
        {
            p[bit / 8U] &= (uint8_t)~mask;
        }
    }
}

static inline uint32_t s2_get_bits(const uint8_t * p, unsigned offset,
                                   unsigned width)
{
    uint32_t value = 0U;
    for (unsigned i = 0U; i < width; ++i)
    {
        unsigned bit = offset + i;
        value |= (((uint32_t)p[bit / 8U] >> (bit % 8U)) & 1U) << i;
    }
    return value;
}

/* Sign-extend a 'width'-bit two's complement field without relying on
 * implementation-defined conversions.  'width' is 1..31. */
static inline int32_t s2_sign_extend(uint32_t raw, unsigned width)
{
    uint32_t sign = 1UL << (width - 1U);
    return ((raw & sign) != 0U)
               ? (int32_t)((int64_t)raw - ((int64_t)1 << width))
               : (int32_t)raw;
}

static inline bool s2_finite(float v)
{
    return v >= -FLT_MAX && v <= FLT_MAX;
}

/* Round half away from zero and clamp to [lo, hi].  'v' must be finite. */
static inline int32_t s2_round_saturate(float v, int32_t lo, int32_t hi)
{
    if (v >= (float)hi)
    {
        return hi;
    }
    if (v <= (float)lo)
    {
        return lo;
    }
    return (int32_t)((v >= 0.0f) ? (v + 0.5f) : (v - 0.5f));
}

/*
 * Ticks travelled from 'before' to 'now', both as unpacked (16-bit) or as
 * full int32 counts.  Correct across the 16-bit wrap as long as the wheel
 * moved fewer than 32,768 ticks between the two reads.
 */
static inline int32_t s2_tick_delta(int32_t now, int32_t before)
{
    uint32_t d = ((uint32_t)now - (uint32_t)before) & 0xFFFFU;
    return (d >= 0x8000U) ? (int32_t)d - 0x10000 : (int32_t)d;
}

/* ---- command ----------------------------------------------------------- */

static inline bool s2_command_valid(const MOTION_COMMAND * c)
{
    if (!c || !s2_finite(c->value))
    {
        return false;
    }
    switch (c->type)
    {
        case MOTION_CMD_STOP:
        case MOTION_CMD_RESET_FAULT: return c->value == 0;
        case MOTION_CMD_FORWARD_CM:
        case MOTION_CMD_TURN_LEFT_DEG:
        case MOTION_CMD_TURN_RIGHT_DEG:
            return c->value > 0 && c->value <= 100000;
        case MOTION_CMD_FOLLOW_LINE: return c->value >= -1 && c->value <= 1;
        default: return false;
    }
}

/* byte 0: command bits 0..2; bits 3..7 reserved zero. bytes 1..4: value. */
static inline bool s2_command_pack(S2_PACKED_COMMAND *    p,
                                   const MOTION_COMMAND * c)
{
    if (!p || !s2_command_valid(c))
    {
        return false;
    }
    p->data[0] = (uint8_t)c->type;
    s2_put_float(p->data + 1, c->value);
    return true;
}

static inline bool s2_command_unpack(MOTION_COMMAND *          c,
                                     const S2_PACKED_COMMAND * p)
{
    if (!p || !c || (p->data[0] & 0xf8))
    {
        return false;
    }
    MOTION_COMMAND tmp = {(MOTION_COMMAND_TYPE)p->data[0],
                          s2_get_float(p->data + 1)};
    if (!s2_command_valid(&tmp))
    {
        return false;
    }
    *c = tmp;
    return true;
}

/* ---- status ------------------------------------------------------------ */

/*
 * Pack a status snapshot.  Returns false (and leaves *p unspecified) for an
 * out-of-range enum or a non-finite number.  Finite numbers beyond a field's
 * range saturate; encoder counts wrap.  Bit 95 is reserved zero.
 */
static inline bool s2_status_pack(S2_PACKED_STATUS * p, const MOTION_STATUS * s)
{
    if (!p || !s || (unsigned)s->state > 6U ||
        (unsigned)s->active_command > 6U || (unsigned)s->fault > 3U ||
        !s2_finite(s->left_speed_cm_s) || !s2_finite(s->right_speed_cm_s) ||
        !s2_finite(s->distance_cm) || !s2_finite(s->left_pwm_percent) ||
        !s2_finite(s->right_pwm_percent))
    {
        return false;
    }
    for (unsigned i = 0U; i < S2_STATUS_BYTES; ++i)
    {
        p->data[i] = 0U;
    }
    s2_put_bits(p->data, S2_STATUS_BIT_STATE, 3U, (uint32_t)s->state);
    s2_put_bits(p->data, S2_STATUS_BIT_COMMAND, 3U,
                (uint32_t)s->active_command);
    s2_put_bits(p->data, S2_STATUS_BIT_FAULT, 2U, (uint32_t)s->fault);
    s2_put_bits(p->data, S2_STATUS_BIT_COMPLETE, 1U,
                s->motion_complete ? 1U : 0U);
    s2_put_bits(p->data, S2_STATUS_BIT_LEFT_TICKS, S2_STATUS_TICK_BITS,
                (uint32_t)s->left_encoder_ticks);
    s2_put_bits(p->data, S2_STATUS_BIT_RIGHT_TICKS, S2_STATUS_TICK_BITS,
                (uint32_t)s->right_encoder_ticks);
    s2_put_bits(p->data, S2_STATUS_BIT_LEFT_SPEED, S2_STATUS_SPEED_BITS,
                (uint32_t)s2_round_saturate(s->left_speed_cm_s,
                                            S2_STATUS_SPEED_MIN,
                                            S2_STATUS_SPEED_MAX));
    s2_put_bits(p->data, S2_STATUS_BIT_RIGHT_SPEED, S2_STATUS_SPEED_BITS,
                (uint32_t)s2_round_saturate(s->right_speed_cm_s,
                                            S2_STATUS_SPEED_MIN,
                                            S2_STATUS_SPEED_MAX));
    s2_put_bits(p->data, S2_STATUS_BIT_DISTANCE, S2_STATUS_DISTANCE_BITS,
                (uint32_t)s2_round_saturate(s->distance_cm,
                                            S2_STATUS_DISTANCE_MIN,
                                            S2_STATUS_DISTANCE_MAX));
    s2_put_bits(p->data, S2_STATUS_BIT_LEFT_PWM, S2_STATUS_PWM_BITS,
                (uint32_t)s2_round_saturate(
                    s->left_pwm_percent * S2_STATUS_PWM_SCALE,
                    -S2_STATUS_PWM_RAW_MAX, S2_STATUS_PWM_RAW_MAX));
    s2_put_bits(p->data, S2_STATUS_BIT_RIGHT_PWM, S2_STATUS_PWM_BITS,
                (uint32_t)s2_round_saturate(
                    s->right_pwm_percent * S2_STATUS_PWM_SCALE,
                    -S2_STATUS_PWM_RAW_MAX, S2_STATUS_PWM_RAW_MAX));
    return true;
}

/*
 * Unpack a status packet.  Encoder ticks come back as the sign-extended
 * 16-bit value: compare two of them with s2_tick_delta(), never directly.
 * Rejects (and leaves *s unchanged) reserved enum codes, a set reserved bit
 * and a PWM outside -1000..1000.
 */
static inline bool s2_status_unpack(MOTION_STATUS *          s,
                                    const S2_PACKED_STATUS * p)
{
    if (!p || !s)
    {
        return false;
    }
    uint32_t state    = s2_get_bits(p->data, S2_STATUS_BIT_STATE, 3U);
    uint32_t command  = s2_get_bits(p->data, S2_STATUS_BIT_COMMAND, 3U);
    int32_t  left_pwm = s2_sign_extend(
         s2_get_bits(p->data, S2_STATUS_BIT_LEFT_PWM, S2_STATUS_PWM_BITS),
         S2_STATUS_PWM_BITS);
    int32_t right_pwm = s2_sign_extend(
        s2_get_bits(p->data, S2_STATUS_BIT_RIGHT_PWM, S2_STATUS_PWM_BITS),
        S2_STATUS_PWM_BITS);
    if (state > 6U || command > 6U ||
        s2_get_bits(p->data, S2_STATUS_BITS_USED,
                    8U * S2_STATUS_BYTES - S2_STATUS_BITS_USED) != 0U ||
        left_pwm < -S2_STATUS_PWM_RAW_MAX || left_pwm > S2_STATUS_PWM_RAW_MAX ||
        right_pwm < -S2_STATUS_PWM_RAW_MAX || right_pwm > S2_STATUS_PWM_RAW_MAX)
    {
        return false;
    }
    MOTION_STATUS tmp = {
        .state          = (MOTION_STATE)state,
        .active_command = (MOTION_COMMAND_TYPE)command,
        .fault = (MOTION_FAULT)s2_get_bits(p->data, S2_STATUS_BIT_FAULT, 2U),
        .motion_complete =
            s2_get_bits(p->data, S2_STATUS_BIT_COMPLETE, 1U) != 0U,
        .left_encoder_ticks = s2_sign_extend(
            s2_get_bits(p->data, S2_STATUS_BIT_LEFT_TICKS, S2_STATUS_TICK_BITS),
            S2_STATUS_TICK_BITS),
        .right_encoder_ticks =
            s2_sign_extend(s2_get_bits(p->data, S2_STATUS_BIT_RIGHT_TICKS,
                                       S2_STATUS_TICK_BITS),
                           S2_STATUS_TICK_BITS),
        .left_speed_cm_s =
            (float)s2_sign_extend(s2_get_bits(p->data, S2_STATUS_BIT_LEFT_SPEED,
                                              S2_STATUS_SPEED_BITS),
                                  S2_STATUS_SPEED_BITS),
        .right_speed_cm_s = (float)s2_sign_extend(
            s2_get_bits(p->data, S2_STATUS_BIT_RIGHT_SPEED,
                        S2_STATUS_SPEED_BITS),
            S2_STATUS_SPEED_BITS),
        .distance_cm =
            (float)s2_sign_extend(s2_get_bits(p->data, S2_STATUS_BIT_DISTANCE,
                                              S2_STATUS_DISTANCE_BITS),
                                  S2_STATUS_DISTANCE_BITS),
        .left_pwm_percent  = (float)left_pwm / S2_STATUS_PWM_SCALE,
        .right_pwm_percent = (float)right_pwm / S2_STATUS_PWM_SCALE,
    };
    *s = tmp;
    return true;
}

#endif /* S2_MOTION_PROTOCOL_H */
