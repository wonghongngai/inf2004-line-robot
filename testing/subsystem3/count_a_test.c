/* Legacy mode: cc -DAPP_ALL_TEST=1 -DLINE_BARCODE_COUNT_A=1 -Iapp_program/subsystem3 testing/subsystem3/count_a_test.c app_program/subsystem3/line_barcode.c -o /tmp/count-a-test */
#include "line_barcode.h"
#include <assert.h>
#include <stdio.h>
static unsigned hits;
static void sample(line_barcode_ctx_t *ctx, unsigned sensors, unsigned ticks)
{
    while (ticks--) {
        line_barcode_msg_t msg = line_barcode_step(ctx, sensors, 10u);
        if (((msg.data[1] >> 3u) & 7u) == LINE_BARCODE_EVT_BARCODE) {
            assert((msg.data[2] & LINE_BARCODE_LETTER_MASK) == 0u);
            ++hits;
        }
    }
}
static void scan(line_barcode_ctx_t *ctx, unsigned runs)
{
    sample(ctx, 2u, 15u);
    for (unsigned i = 0; i < runs; ++i)
        sample(ctx, 2u | ((i % 2u == 0u) ? 4u : 0u), 4u);
    sample(ctx, 2u, 15u);
}
int main(void)
{
    line_barcode_ctx_t ctx;
    line_barcode_reset(&ctx);
    scan(&ctx, 27u); assert(hits == 0u);
    scan(&ctx, 29u); assert(hits == 1u);
    sample(&ctx, 2u, 100u); assert(hits == 1u);
    scan(&ctx, 31u); assert(hits == 1u);
    scan(&ctx, 29u); assert(hits == 2u);
    line_barcode_set_layout(&ctx, LINE_BARCODE_LAYOUT_TRACK);
    scan(&ctx, 29u); assert(hits == 2u);
    puts("PASS: 29 segments, partial/long rejection, no repeats, rearm, TRACK disabled");
}
