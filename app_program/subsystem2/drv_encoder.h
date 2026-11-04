/*
 * drv_encoder.h
 * Buddy 2 wheel encoder input driver.
 */
#ifndef S2_DRV_ENCODER_H
#define S2_DRV_ENCODER_H

#include "../app.h"
#include <stdint.h>

/*
 * Initialise the four encoder GPIO inputs.
 */
ER drv_encoder_init(void);

/*
 * Read encoder A/B levels and update quadrature counters.
 *
 * Currently called every 1 ms by Motion Control as a fallback until the team
 * defines a shared GPIO interrupt dispatcher.
 */
void drv_encoder_poll(void);

/*
 * Copy current encoder counts.
 */
void drv_encoder_get_counts(
    int32_t *left_ticks,
    int32_t *right_ticks
);

/*
 * Reset both counters to zero.
 */
void drv_encoder_reset(void);

#endif /* S2_DRV_ENCODER_H */