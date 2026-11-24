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

static inline void s2_put_u32(uint8_t * p_buf, uint32_t value)
{
    for (unsigned idx = 0U; idx < 4U; ++idx)
    {
        p_buf[idx] = (uint8_t)(value >> (8U * idx));
    }
}

static inline uint32_t s2_get_u32(const uint8_t * p_buf)
{
    return (uint32_t)p_buf[0] | ((uint32_t)p_buf[1] << 8) |
           ((uint32_t)p_buf[2] << 16) | ((uint32_t)p_buf[3] << 24);
}

static inline void s2_put_float(uint8_t * p_buf, float value)
{
    union
    {
        float    f;
        uint32_t u;
    } bits = {.f = value};
    s2_put_u32(p_buf, bits.u);
}

static inline float s2_get_float(const uint8_t * p_buf)
{
    union
    {
        float    f;
        uint32_t u;
    } bits = {.u = s2_get_u32(p_buf)};
    return bits.f;
}

/* Write the low 'width' bits of 'value' at bit 'offset' (little-endian). */
static inline void s2_put_bits(uint8_t * p_buf, unsigned offset, unsigned width,
                               uint32_t value)
{
    for (unsigned idx = 0U; idx < width; ++idx)
    {
        unsigned bit  = offset + idx;
        uint8_t  mask = (uint8_t)(1U << (bit % 8U));
        if (((value >> idx) & 1U) != 0U)
        {
            p_buf[bit / 8U] |= mask;
        }
        else
        {
            p_buf[bit / 8U] &= (uint8_t)~mask;
        }
    }
}

static inline uint32_t s2_get_bits(const uint8_t * p_buf, unsigned offset,
                                   unsigned width)
{
    uint32_t value = 0U;
    for (unsigned idx = 0U; idx < width; ++idx)
    {
        unsigned bit = offset + idx;
        value |= (((uint32_t)p_buf[bit / 8U] >> (bit % 8U)) & 1U) << idx;
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

static inline bool s2_finite(float value)
{
    return value >= -FLT_MAX && value <= FLT_MAX;
}

/* Round half away from zero and clamp to [lo, hi].  'v' must be finite. */
static inline int32_t s2_round_saturate(float value, int32_t low, int32_t high)
{
    if (value >= (float)high)
    {
        return high;
    }
    if (value <= (float)low)
    {
        return low;
    }
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

/*
 * Ticks travelled from 'before' to 'now', both as unpacked (16-bit) or as
 * full int32 counts.  Correct across the 16-bit wrap as long as the wheel
 * moved fewer than 32,768 ticks between the two reads.
 */
static inline int32_t s2_tick_delta(int32_t now, int32_t before)
{
    uint32_t delta = ((uint32_t)now - (uint32_t)before) & 0xFFFFU;
    return (delta >= 0x8000U) ? (int32_t)delta - 0x10000 : (int32_t)delta;
}

/* ---- command ----------------------------------------------------------- */

static inline bool s2_command_valid(const MOTION_COMMAND * p_cmd)
{
    if (!p_cmd || !s2_finite(p_cmd->value))
    {
        return false;
    }
    switch (p_cmd->type)
    {
        case MOTION_CMD_STOP:
        case MOTION_CMD_RESET_FAULT: return p_cmd->value == 0;
        case MOTION_CMD_FORWARD_CM:
        case MOTION_CMD_TURN_LEFT_DEG:
        case MOTION_CMD_TURN_RIGHT_DEG:
            return p_cmd->value > 0 && p_cmd->value <= 100000;
        case MOTION_CMD_FOLLOW_LINE:
            return p_cmd->value >= -1 && p_cmd->value <= 1;
        default: return false;
    }
}

/* byte 0: command bits 0..2; bits 3..7 reserved zero. bytes 1..4: value. */
static inline bool s2_command_pack(S2_PACKED_COMMAND *    p_buf,
                                   const MOTION_COMMAND * p_cmd)
{
    if (!p_buf || !s2_command_valid(p_cmd))
    {
        return false;
    }
    p_buf->data[0] = (uint8_t)p_cmd->type;
    s2_put_float(p_buf->data + 1, p_cmd->value);
    return true;
}

static inline bool s2_command_unpack(MOTION_COMMAND *          p_cmd,
                                     const S2_PACKED_COMMAND * p_buf)
{
    if (!p_buf || !p_cmd || (p_buf->data[0] & 0xf8))
    {
        return false;
    }
    MOTION_COMMAND tmp = {(MOTION_COMMAND_TYPE)p_buf->data[0],
                          s2_get_float(p_buf->data + 1)};
    if (!s2_command_valid(&tmp))
    {
        return false;
    }
    *p_cmd = tmp;
    return true;
}

/* ---- status ------------------------------------------------------------ */

/*
 * Pack a status snapshot.  Returns false (and leaves *p unspecified) for an
 * out-of-range enum or a non-finite number.  Finite numbers beyond a field's
 * range saturate; encoder counts wrap.  Bit 95 is reserved zero.
 */
static inline bool s2_status_pack(S2_PACKED_STATUS *    p_buf,
                                  const MOTION_STATUS * p_status)
{
    if (!p_buf || !p_status || (unsigned)p_status->state > 6U ||
        (unsigned)p_status->active_command > 6U ||
        (unsigned)p_status->fault > 3U ||
        !s2_finite(p_status->left_speed_cm_s) ||
        !s2_finite(p_status->right_speed_cm_s) ||
        !s2_finite(p_status->distance_cm) ||
        !s2_finite(p_status->left_pwm_percent) ||
        !s2_finite(p_status->right_pwm_percent))
    {
        return false;
    }
    for (unsigned idx = 0U; idx < S2_STATUS_BYTES; ++idx)
    {
        p_buf->data[idx] = 0U;
    }
    s2_put_bits(p_buf->data, S2_STATUS_BIT_STATE, 3U,
                (uint32_t)p_status->state);
    s2_put_bits(p_buf->data, S2_STATUS_BIT_COMMAND, 3U,
                (uint32_t)p_status->active_command);
    s2_put_bits(p_buf->data, S2_STATUS_BIT_FAULT, 2U,
                (uint32_t)p_status->fault);
    s2_put_bits(p_buf->data, S2_STATUS_BIT_COMPLETE, 1U,
                p_status->motion_complete ? 1U : 0U);
    s2_put_bits(p_buf->data, S2_STATUS_BIT_LEFT_TICKS, S2_STATUS_TICK_BITS,
                (uint32_t)p_status->left_encoder_ticks);
    s2_put_bits(p_buf->data, S2_STATUS_BIT_RIGHT_TICKS, S2_STATUS_TICK_BITS,
                (uint32_t)p_status->right_encoder_ticks);
    s2_put_bits(p_buf->data, S2_STATUS_BIT_LEFT_SPEED, S2_STATUS_SPEED_BITS,
                (uint32_t)s2_round_saturate(p_status->left_speed_cm_s,
                                            S2_STATUS_SPEED_MIN,
                                            S2_STATUS_SPEED_MAX));
    s2_put_bits(p_buf->data, S2_STATUS_BIT_RIGHT_SPEED, S2_STATUS_SPEED_BITS,
                (uint32_t)s2_round_saturate(p_status->right_speed_cm_s,
                                            S2_STATUS_SPEED_MIN,
                                            S2_STATUS_SPEED_MAX));
    s2_put_bits(p_buf->data, S2_STATUS_BIT_DISTANCE, S2_STATUS_DISTANCE_BITS,
                (uint32_t)s2_round_saturate(p_status->distance_cm,
                                            S2_STATUS_DISTANCE_MIN,
                                            S2_STATUS_DISTANCE_MAX));
    s2_put_bits(p_buf->data, S2_STATUS_BIT_LEFT_PWM, S2_STATUS_PWM_BITS,
                (uint32_t)s2_round_saturate(
                    p_status->left_pwm_percent * S2_STATUS_PWM_SCALE,
                    -S2_STATUS_PWM_RAW_MAX, S2_STATUS_PWM_RAW_MAX));
    s2_put_bits(p_buf->data, S2_STATUS_BIT_RIGHT_PWM, S2_STATUS_PWM_BITS,
                (uint32_t)s2_round_saturate(
                    p_status->right_pwm_percent * S2_STATUS_PWM_SCALE,
                    -S2_STATUS_PWM_RAW_MAX, S2_STATUS_PWM_RAW_MAX));
    return true;
}

/*
 * Unpack a status packet.  Encoder ticks come back as the sign-extended
 * 16-bit value: compare two of them with s2_tick_delta(), never directly.
 * Rejects (and leaves *s unchanged) reserved enum codes, a set reserved bit
 * and a PWM outside -1000..1000.
 */
static inline bool s2_status_unpack(MOTION_STATUS *          p_status,
                                    const S2_PACKED_STATUS * p_buf)
{
    if (!p_buf || !p_status)
    {
        return false;
    }
    uint32_t state    = s2_get_bits(p_buf->data, S2_STATUS_BIT_STATE, 3U);
    uint32_t command  = s2_get_bits(p_buf->data, S2_STATUS_BIT_COMMAND, 3U);
    int32_t  left_pwm = s2_sign_extend(
         s2_get_bits(p_buf->data, S2_STATUS_BIT_LEFT_PWM, S2_STATUS_PWM_BITS),
         S2_STATUS_PWM_BITS);
    int32_t right_pwm = s2_sign_extend(
        s2_get_bits(p_buf->data, S2_STATUS_BIT_RIGHT_PWM, S2_STATUS_PWM_BITS),
        S2_STATUS_PWM_BITS);
    if (state > 6U || command > 6U ||
        s2_get_bits(p_buf->data, S2_STATUS_BITS_USED,
                    8U * S2_STATUS_BYTES - S2_STATUS_BITS_USED) != 0U ||
        left_pwm < -S2_STATUS_PWM_RAW_MAX || left_pwm > S2_STATUS_PWM_RAW_MAX ||
        right_pwm < -S2_STATUS_PWM_RAW_MAX || right_pwm > S2_STATUS_PWM_RAW_MAX)
    {
        return false;
    }
    MOTION_STATUS tmp = {
        .state          = (MOTION_STATE)state,
        .active_command = (MOTION_COMMAND_TYPE)command,
        .fault =
            (MOTION_FAULT)s2_get_bits(p_buf->data, S2_STATUS_BIT_FAULT, 2U),
        .motion_complete =
            s2_get_bits(p_buf->data, S2_STATUS_BIT_COMPLETE, 1U) != 0U,
        .left_encoder_ticks =
            s2_sign_extend(s2_get_bits(p_buf->data, S2_STATUS_BIT_LEFT_TICKS,
                                       S2_STATUS_TICK_BITS),
                           S2_STATUS_TICK_BITS),
        .right_encoder_ticks =
            s2_sign_extend(s2_get_bits(p_buf->data, S2_STATUS_BIT_RIGHT_TICKS,
                                       S2_STATUS_TICK_BITS),
                           S2_STATUS_TICK_BITS),
        .left_speed_cm_s = (float)s2_sign_extend(
            s2_get_bits(p_buf->data, S2_STATUS_BIT_LEFT_SPEED,
                        S2_STATUS_SPEED_BITS),
            S2_STATUS_SPEED_BITS),
        .right_speed_cm_s = (float)s2_sign_extend(
            s2_get_bits(p_buf->data, S2_STATUS_BIT_RIGHT_SPEED,
                        S2_STATUS_SPEED_BITS),
            S2_STATUS_SPEED_BITS),
        .distance_cm = (float)s2_sign_extend(
            s2_get_bits(p_buf->data, S2_STATUS_BIT_DISTANCE,
                        S2_STATUS_DISTANCE_BITS),
            S2_STATUS_DISTANCE_BITS),
        .left_pwm_percent  = (float)left_pwm / S2_STATUS_PWM_SCALE,
        .right_pwm_percent = (float)right_pwm / S2_STATUS_PWM_SCALE,
    };
    *p_status = tmp;
    return true;
}

#endif /* S2_MOTION_PROTOCOL_H */
