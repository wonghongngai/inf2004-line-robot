#include "line_barcode.h"
#include "fixtures/pptx_samples.h"
#include <assert.h>
#include <stdio.h>
static unsigned hits;
static unsigned expected;
static void feed(line_barcode_ctx_t *ctx, unsigned sensors, unsigned ms)
{
    while (ms >= 10u) {
        line_barcode_msg_t msg = line_barcode_step(ctx, sensors, 10u);
        if (((msg.data[1] >> 3u) & 7u) == LINE_BARCODE_EVT_BARCODE) {
            assert((msg.data[2] & LINE_BARCODE_LETTER_MASK) == expected);
            ++hits;
        }
        ms -= 10u;
    }
}
int main(void)
{
    line_barcode_map_t map = {{0u}};
    assert(line_barcode_map_set(&map, 'A', LINE_BARCODE_CMD_LEFT));
    assert(line_barcode_map_set(&map, 'F', LINE_BARCODE_CMD_RIGHT));
    assert(line_barcode_map_set(&map, 'Z', LINE_BARCODE_CMD_STRAIGHT));
    assert(line_barcode_map_set(&map, 'W', LINE_BARCODE_CMD_UTURN));
    for (unsigned l = 0; l < 26u; ++l) {
        uint8_t command = 255u;
        bool mapped = l == 'A'-'A' || l == 'F'-'A' || l == 'Z'-'A' || l == 'W'-'A';
        assert(line_barcode_map_get(&map, l, &command) == mapped);
        if (mapped) assert(command == (l == 0u ? LINE_BARCODE_CMD_LEFT :
            l == 'F'-'A' ? LINE_BARCODE_CMD_RIGHT :
            l == 'Z'-'A' ? LINE_BARCODE_CMD_STRAIGHT : LINE_BARCODE_CMD_UTURN));
        line_barcode_nav_t nav = {0u};
        line_barcode_msg_t msg = line_barcode_pack(2u, 2u, 0u,
            LINE_BARCODE_FOLLOW, LINE_BARCODE_EVT_BARCODE, 0u);
        msg.data[2] = l;
        assert(!line_barcode_nav_update(&nav, &map, &msg));
        msg = line_barcode_pack(7u, 5u, 0u, LINE_BARCODE_JUNCTION,
            LINE_BARCODE_EVT_JUNCTION, 0u);
        assert(line_barcode_nav_update(&nav, &map, &msg) == mapped);
    }
    puts("PASS: A/F/Z/W junction actions; other 22 letters have no action");
    for (unsigned s = 0; s < sizeof(pptx_samples)/sizeof(pptx_samples[0]); ++s) {
        for (unsigned reverse = 0; reverse < 2; ++reverse) {
            uint16_t runs[29];
            for (unsigned i = 0; i < 29; ++i)
                runs[i] = 60u * pptx_samples[s].widths[reverse ? 28u-i : i];
            uint8_t letter;
            expected = pptx_samples[s].letter - 'A';
            assert(line_barcode_decode(runs, 29u, &letter));
            assert(letter == expected);
            line_barcode_ctx_t ctx;
            line_barcode_reset(&ctx);
            unsigned before = hits;
            feed(&ctx, 2u, 700u);
            for (unsigned i = 0; i < 29u; ++i)
                feed(&ctx, 2u | ((i % 2u == 0u) ? 4u : 0u), runs[i]);
            feed(&ctx, 2u, 700u);
            assert(hits == before + 1u);
        }
        printf("PASS: slide %c forward/reverse decode and sampled scan\n", pptx_samples[s].letter);
    }
    uint16_t uniform[29]; uint8_t letter;
    for (unsigned i=0; i<29; ++i) uniform[i]=60u;
    assert(!line_barcode_decode(uniform,29u,&letter));
    puts("PASS: 29 uniform segments rejected");
}
