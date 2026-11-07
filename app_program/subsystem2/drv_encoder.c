/*
 * drv_encoder.c
 * Buddy 2 -- quadrature wheel-encoder reader.
 *
 * Team pin assignment:
 *
 *   Left encoder:
 *      A = GP26
 *      B = GP6
 *
 *   Right encoder:
 *      A = GP2
 *      B = GP3
 *
 * Counting is interrupt driven: every rising or falling edge on any of the
 * four pins raises IO_IRQ_BANK0, and the handler samples both wheels' A/B
 * levels and runs them through the quadrature table.  Buddy 2 owns
 * IO_IRQ_BANK0 (TEAM.md); no other subsystem uses GPIO interrupts.
 *
 * 2026-11-06: the earlier version polled the pins every 1 ms from a task.
 * That missed edges once a wheel turned quickly: the count per real cm
 * depended on speed (about 104 ticks/cm pushed slowly by hand, about 77
 * driven at 12.5 % PWM) and the left wheel read BACKWARDS above about 15 %
 * PWM, so heading control drove the robot in curves.
 *
 * drv_encoder_poll() is kept for its callers but does nothing now.
 */
#include "drv_encoder.h"
#include <tk/syslib.h>

#define ENCODER_LEFT_A_PIN   26U
#define ENCODER_LEFT_B_PIN    6U

#define ENCODER_RIGHT_A_PIN   2U
#define ENCODER_RIGHT_B_PIN   3U

/* RP2040 IO_BANK0 interrupt registers (datasheet 2.19.6.1).  Each GPIO has
 * four bits per register: level low, level high, edge low, edge high. */
#define ENC_IO_BANK0_BASE    0x40014000UL
#define ENC_INTR(n)          (ENC_IO_BANK0_BASE + 0x0F0UL + (4UL * (n)))
#define ENC_PROC0_INTE(n)    (ENC_IO_BANK0_BASE + 0x100UL + (4UL * (n)))
#define ENC_PROC1_INTE(n)    (ENC_IO_BANK0_BASE + 0x130UL + (4UL * (n)))
#define ENC_EDGE_BITS(pin)   (0xCUL << (4U * ((pin) % 8U))) /* both edges */
#define ENC_REG_INDEX(pin)   ((pin) / 8U)

/* SIO: GPIO input levels, and which core is running */
#define ENC_SIO_GPIO_IN      0xD0000004UL
#define ENC_SIO_CPUID        0xD0000000UL

#define ENC_INTNO_IO_BANK0   13U /* IO_IRQ_BANK0 */
#define ENC_INTPRI           2   /* same level as the physical timer */

/*
 * State belonging to one wheel encoder.  'count' is written only by the
 * interrupt handler (one aligned 32-bit store); 'zero' only by tasks
 * (drv_encoder_reset).  Readers see count - zero.
 */
typedef struct {

    volatile int32_t count;

    volatile int32_t zero;

    UINT previous_ab;

} ENCODER_CHANNEL;

static ENCODER_CHANNEL left_encoder;

static ENCODER_CHANNEL right_encoder;

static BOOL irq_defined = FALSE;

/*
 * Extract one wheel's A and B from a GPIO_IN snapshot:
 *
 *     bit 1 = A
 *     bit 0 = B
 */
static UINT ab_from_levels(
    UW levels,
    UINT pin_a,
    UINT pin_b
)
{
    return
        (((levels >> pin_a) & 1U) << 1) |
        ((levels >> pin_b) & 1U);
}

/*
 * Read A and B now and combine as above.
 */
static UINT read_ab(
    UINT pin_a,
    UINT pin_b
)
{
    return ab_from_levels(in_w(ENC_SIO_GPIO_IN), pin_a, pin_b);
}

/*
 * Quadrature transition lookup.
 *
 * Valid encoder sequences produce +1 or -1.
 *
 * Invalid two-bit jumps produce 0 instead of inventing movement.
 */
static int32_t quadrature_delta(
    UINT previous_ab,
    UINT current_ab
)
{
    static const int8_t transition[16] = {

         0, -1,  1,  0,

         1,  0,  0, -1,

        -1,  0,  0,  1,

         0,  1, -1,  0
    };

    return transition[
        ((previous_ab & 3U) << 2) |
        (current_ab & 3U)
    ];
}

/*
 * Process one encoder A/B sample (interrupt handler only).
 */
static void update_channel(
    ENCODER_CHANNEL *channel,
    UINT current_ab
)
{
    channel->count +=
        quadrature_delta(
            channel->previous_ab,
            current_ab
        );

    channel->previous_ab =
        current_ab;
}

/*
 * Clear the latched edge bits of the four encoder pins.
 */
static void clear_edges(void)
{
    out_w(ENC_INTR(ENC_REG_INDEX(ENCODER_LEFT_A_PIN)),
          ENC_EDGE_BITS(ENCODER_LEFT_A_PIN));
    out_w(ENC_INTR(ENC_REG_INDEX(ENCODER_LEFT_B_PIN)),
          ENC_EDGE_BITS(ENCODER_LEFT_B_PIN));
    out_w(ENC_INTR(ENC_REG_INDEX(ENCODER_RIGHT_A_PIN)),
          ENC_EDGE_BITS(ENCODER_RIGHT_A_PIN));
    out_w(ENC_INTR(ENC_REG_INDEX(ENCODER_RIGHT_B_PIN)),
          ENC_EDGE_BITS(ENCODER_RIGHT_B_PIN));
}

/*
 * IO_IRQ_BANK0 handler: an edge on an encoder pin.  Clear the edges
 * first, then sample, so an edge arriving meanwhile raises the interrupt
 * again instead of being lost.  The quadrature table ignores a wheel whose
 * levels did not change.
 */
static void encoder_inthdr(UINT intno)
{
    UW levels;

    (void)intno;

    clear_edges();
    levels = in_w(ENC_SIO_GPIO_IN);

    update_channel(&left_encoder,
                   ab_from_levels(levels, ENCODER_LEFT_A_PIN,
                                  ENCODER_LEFT_B_PIN));
    update_channel(&right_encoder,
                   ab_from_levels(levels, ENCODER_RIGHT_A_PIN,
                                  ENCODER_RIGHT_B_PIN));
}

/*
 * Enable edge interrupts for one pin on the core running this code.
 */
static void enable_pin_edges(UINT pin, BOOL b_core1)
{
    UW reg = b_core1 ? ENC_PROC1_INTE(ENC_REG_INDEX(pin))
                     : ENC_PROC0_INTE(ENC_REG_INDEX(pin));

    out_w(reg, in_w(reg) | ENC_EDGE_BITS(pin));
}

ER drv_encoder_init(void)
{
    ER     er;
    T_DINT dint;
    BOOL   b_core1;

    /*
     * Left encoder inputs.
     */
    er = gpio_set_pin(
        ENCODER_LEFT_A_PIN,
        GPIO_MODE_IN
    );

    if (er < E_OK) {
        return er;
    }

    er = gpio_set_pin(
        ENCODER_LEFT_B_PIN,
        GPIO_MODE_IN
    );

    if (er < E_OK) {
        return er;
    }

    /*
     * Right encoder inputs.
     */
    er = gpio_set_pin(
        ENCODER_RIGHT_A_PIN,
        GPIO_MODE_IN
    );

    if (er < E_OK) {
        return er;
    }

    er = gpio_set_pin(
        ENCODER_RIGHT_B_PIN,
        GPIO_MODE_IN
    );

    if (er < E_OK) {
        return er;
    }

    /*
     * Start counters from zero, and remember the current A/B values so
     * the first edge has a valid previous state.
     */
    left_encoder.count = 0;
    left_encoder.zero = 0;
    right_encoder.count = 0;
    right_encoder.zero = 0;

    left_encoder.previous_ab =
        read_ab(
            ENCODER_LEFT_A_PIN,
            ENCODER_LEFT_B_PIN
        );

    right_encoder.previous_ab =
        read_ab(
            ENCODER_RIGHT_A_PIN,
            ENCODER_RIGHT_B_PIN
        );

    if (irq_defined) {
        return E_OK; /* already counting */
    }

    /*
     * Edge interrupts on all four pins, handled on this core.
     */
    dint.intatr = TA_HLNG;
    dint.inthdr = (FP)encoder_inthdr;
    er = tk_def_int(ENC_INTNO_IO_BANK0, &dint);
    if (er < E_OK) {
        return er;
    }

    b_core1 = (in_w(ENC_SIO_CPUID) != 0U) ? TRUE : FALSE;
    clear_edges();
    enable_pin_edges(ENCODER_LEFT_A_PIN, b_core1);
    enable_pin_edges(ENCODER_LEFT_B_PIN, b_core1);
    enable_pin_edges(ENCODER_RIGHT_A_PIN, b_core1);
    enable_pin_edges(ENCODER_RIGHT_B_PIN, b_core1);
    EnableInt(ENC_INTNO_IO_BANK0, ENC_INTPRI);
    irq_defined = TRUE;

    return E_OK;
}

void drv_encoder_poll(void)
{
    /* Counting is interrupt driven now: nothing to do.  Kept so the
     * existing 1 ms callers still build. */
}

void drv_encoder_get_counts(
    int32_t *left_ticks,
    int32_t *right_ticks
)
{
    if (left_ticks != 0) {

        *left_ticks =
            left_encoder.count - left_encoder.zero;
    }

    if (right_ticks != 0) {

        *right_ticks =
            right_encoder.count - right_encoder.zero;
    }
}

void drv_encoder_reset(void)
{
    /*
     * The handler owns 'count' (it may run on the other core), so reset
     * by moving the zero point instead of writing the count.
     */
    left_encoder.zero = left_encoder.count;

    right_encoder.zero = right_encoder.count;
}
