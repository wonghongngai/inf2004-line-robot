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
 * The current repository has no team-wide GPIO IRQ dispatcher yet.
 *
 * To avoid Buddy 2 privately taking ownership of IO_IRQ_BANK0, this version
 * polls the encoder A/B levels from the single Motion Control task.
 *
 * drv_encoder_poll() is currently called every 1 ms.
 *
 * Hardware validation MUST verify that this rate does not miss encoder edges
 * at maximum wheel speed.
 *
 * If it does, retain the quadrature decoder below but call it using the
 * team's future shared GPIO IRQ dispatcher.
 */
#include "drv_encoder.h"

#define ENCODER_LEFT_A_PIN   26U
#define ENCODER_LEFT_B_PIN    6U

#define ENCODER_RIGHT_A_PIN   2U
#define ENCODER_RIGHT_B_PIN   3U

/*
 * State belonging to one wheel encoder.
 */
typedef struct {

    int32_t count;

    UINT previous_ab;

} ENCODER_CHANNEL;

static ENCODER_CHANNEL left_encoder;

static ENCODER_CHANNEL right_encoder;

/*
 * Read A and B and combine into:
 *
 *     bit 1 = A
 *     bit 0 = B
 *
 * producing values:
 *
 *     00
 *     01
 *     10
 *     11
 */
static UINT read_ab(
    UINT pin_a,
    UINT pin_b
)
{
    return
        ((gpio_get_val(pin_a) & 1U) << 1) |
        (gpio_get_val(pin_b) & 1U);
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
 * Process one encoder A/B sample.
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

ER drv_encoder_init(void)
{
    ER er;

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
     * Start counters from zero.
     */
    left_encoder.count = 0;

    right_encoder.count = 0;

    /*
     * Remember the current A/B values so the first update has a valid
     * previous state.
     */
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

    return E_OK;
}

void drv_encoder_poll(void)
{
    /*
     * Sample and process the left wheel.
     */
    update_channel(
        &left_encoder,
        read_ab(
            ENCODER_LEFT_A_PIN,
            ENCODER_LEFT_B_PIN
        )
    );

    /*
     * Sample and process the right wheel.
     */
    update_channel(
        &right_encoder,
        read_ab(
            ENCODER_RIGHT_A_PIN,
            ENCODER_RIGHT_B_PIN
        )
    );
}

void drv_encoder_get_counts(
    int32_t *left_ticks,
    int32_t *right_ticks
)
{
    if (left_ticks != 0) {

        *left_ticks =
            left_encoder.count;
    }

    if (right_ticks != 0) {

        *right_ticks =
            right_encoder.count;
    }
}

void drv_encoder_reset(void)
{
    left_encoder.count = 0;

    right_encoder.count = 0;

    /*
     * Re-read the current A/B states after resetting.
     */
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
}