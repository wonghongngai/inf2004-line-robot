/** @file line_barcode_test.c
 * @brief The same deterministic core tests run on the host and on the Pico.
 */
#include "line_barcode.h"
#include <stddef.h>

#if APP_TEST
#include "../../testing/subsystem3/fixtures/barcode_samples.h"
#ifdef LINE_BARCODE_HOST_TEST
#include <stdio.h>
static unsigned int g_failures = 0u;
#define TEST_CHECK(ID, CONDITION, TEXT) \
    do \
    { \
        bool b_pass = (CONDITION); \
        printf("[TEST] %s %s: %s\n", ID, b_pass ? "PASS" : "FAIL", TEXT); \
        g_failures += b_pass ? 0u : 1u; \
    } while (0)
#else
#include "../test_report.h"
#endif

#define TEST_SYMBOLS       LINE_BARCODE_ALPHABET_SIZE
#define TEST_DIRECTIONS    2u
#define TEST_ELEMENT_COUNT 9u
#define TEST_SENSOR_COUNT  8u
#define TEST_NARROW_MS     60u
#define TEST_WIDE_MS       180u
#define TEST_QUIET_MS      700u
#define TEST_ZERO_MS       0u

static void build_frame(uint16_t *p_runs, uint8_t command, bool b_reverse)
{
    static uint16_t const symbols[TEST_SYMBOLS] =
        {0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00du,
         0x10cu, 0x04cu, 0x01cu, 0x103u, 0x043u, 0x142u, 0x013u,
         0x112u, 0x052u, 0x007u, 0x106u, 0x046u, 0x016u, 0x181u,
         0x0c1u, 0x1c0u, 0x091u, 0x190u, 0x0d0u};
    uint16_t patterns[3] = {0x094u, symbols[command], 0x094u};
    for (uint8_t idx = 0u; idx < LINE_BARCODE_FRAME_RUNS; idx++)
    {
        uint8_t character = idx / 10u;
        uint8_t element = idx % 10u;
        uint16_t width = TEST_NARROW_MS;
        if ((element < TEST_ELEMENT_COUNT)
            && (0u != (patterns[character] & (1u << (8u - element)))))
        {
            width = TEST_WIDE_MS;
        }
        uint8_t target = b_reverse ? (LINE_BARCODE_FRAME_RUNS - 1u - idx) : idx;
        p_runs[target] = width;
    }
}

static line_barcode_msg_t feed(line_barcode_ctx_t *p_ctx, uint8_t sensors,
                               uint16_t duration_ms, uint8_t *p_commands,
                               uint8_t *p_last_command)
{
    line_barcode_msg_t msg = {{0u, 0u}};
    for (uint16_t elapsed = TEST_ZERO_MS; elapsed < duration_ms;
         elapsed += LINE_BARCODE_PERIOD_MS)
    {
        msg = line_barcode_step(p_ctx, sensors, LINE_BARCODE_PERIOD_MS);
        if (LINE_BARCODE_EVT_BARCODE == ((msg.data[1] >> 3u) & 7u))
        {
            (*p_commands)++;
            *p_last_command = msg.data[2] & LINE_BARCODE_LETTER_MASK;
        }
    }
    return (msg);
}

static bool test_stream(uint8_t command, bool b_reverse, uint16_t narrow_ms)
{
    line_barcode_ctx_t context;
    uint16_t runs[LINE_BARCODE_FRAME_RUNS] = {0u};
    uint8_t commands = 0u;
    uint8_t last_command = 0xffu;
    line_barcode_reset(&context);
    build_frame(runs, command, b_reverse);
    uint16_t quiet_ms = (uint16_t)(10u * narrow_ms + 100u);
    for (uint8_t idx = 0u; idx < LINE_BARCODE_FRAME_RUNS; idx++)
    {
        runs[idx] = (uint16_t)((uint32_t)runs[idx] * narrow_ms
                               / TEST_NARROW_MS);
    }
    (void)feed(&context, 3u, quiet_ms,
               &commands, &last_command);
    for (uint8_t idx = 0u; idx < LINE_BARCODE_FRAME_RUNS; idx++)
    {
        uint8_t sensors = (0u == (idx & 1u)) ? 7u : 3u;
        (void)feed(&context, sensors, runs[idx], &commands, &last_command);
    }
    (void)feed(&context, 3u, quiet_ms,
               &commands, &last_command);
    (void)feed(&context, 3u, quiet_ms,
               &commands, &last_command);
    return ((1u == commands) && (command == last_command));
}

static bool test_pdf_samples(void)
{
    uint8_t letter = 0xffu;
    uint16_t reversed[LINE_BARCODE_FRAME_RUNS] = {0u};
    bool b_valid = line_barcode_decode(g_sample_a, LINE_BARCODE_FRAME_RUNS,
                                       &letter) && (0u == letter);
    b_valid = b_valid && line_barcode_decode(g_sample_z,
        LINE_BARCODE_FRAME_RUNS, &letter) && (25u == letter);
    for (uint8_t idx = 0u; idx < LINE_BARCODE_FRAME_RUNS; idx++)
    {
        reversed[idx] = g_sample_z[LINE_BARCODE_FRAME_RUNS - 1u - idx];
    }
    return (b_valid && line_barcode_decode(reversed,
             LINE_BARCODE_FRAME_RUNS, &letter) && (25u == letter));
}

static uint8_t msg_event(line_barcode_msg_t const *p_msg)
{
    return ((p_msg->data[1] >> 3u) & 7u);
}

static bool test_mapping(void)
{
    line_barcode_map_t mapping = {{0u}};
    line_barcode_nav_t nav = {0u};
    line_barcode_msg_t barcode = line_barcode_pack(3u, 2u, 0u,
        LINE_BARCODE_FOLLOW, LINE_BARCODE_EVT_BARCODE, 0u);
    line_barcode_msg_t junction = line_barcode_pack(7u, 2u, 0u,
        LINE_BARCODE_JUNCTION, LINE_BARCODE_EVT_JUNCTION, 0u);
    line_barcode_msg_t msg = junction;

    /* Unmapped letter: stored, then consumed as a plain junction. */
    barcode.data[2] = 25u;
    bool b_valid = !line_barcode_nav_update(&nav, &mapping, &barcode)
                   && (LINE_BARCODE_EVT_BARCODE == msg_event(&barcode))
                   && !line_barcode_nav_update(&nav, &mapping, &msg)
                   && (LINE_BARCODE_EVT_JUNCTION == msg_event(&msg))
                   && (0u == nav.pending);
    for (uint8_t letter = 0u; letter < LINE_BARCODE_ALPHABET_SIZE; letter++)
    {
        b_valid = b_valid && line_barcode_map_set(&mapping,
            (char)('A' + letter), letter % 4u);
    }
    for (uint8_t letter = 0u; letter < LINE_BARCODE_ALPHABET_SIZE; letter++)
    {
        barcode.data[2] = letter;
        msg = junction;
        b_valid = b_valid && !line_barcode_nav_update(&nav, &mapping, &barcode)
                   && line_barcode_nav_update(&nav, &mapping, &msg)
                   && ((letter % 4u) == (msg.data[0] >> 6u))
                   && (letter == (msg.data[2] & LINE_BARCODE_LETTER_MASK))
                   && (LINE_BARCODE_EVT_COMMAND == msg_event(&msg));
    }
    /* A junction with no letter waiting gives no command. */
    msg = junction;
    b_valid = b_valid && !line_barcode_nav_update(&nav, &mapping, &msg);
    /* Barcodes seen while searching for the line are not stored. */
    barcode.data[1] = (LINE_BARCODE_EVT_BARCODE << 3u) | LINE_BARCODE_SEARCH;
    (void)line_barcode_nav_update(&nav, &mapping, &barcode);
    b_valid = b_valid && (0u == nav.pending);
    line_barcode_map_clear(&mapping);
    return (b_valid && !line_barcode_map_set(&mapping, 'a', 0u)
            && !line_barcode_map_set(&mapping, 'Z', 4u)
            && (10u == sizeof(mapping)));
}

/* Steps one sensor pattern for duration_ms; BARCODE events go through the
 * optional nav so the stored letter is the decoder's real output.
 */
static void feed_nav(line_barcode_ctx_t *p_ctx, uint8_t sensors,
                     uint16_t duration_ms, line_barcode_nav_t *p_nav,
                     uint8_t *p_count, uint8_t *p_letter)
{
    static line_barcode_map_t const empty_map = {{0u}};
    for (uint16_t elapsed = 0u; elapsed < duration_ms;
         elapsed += LINE_BARCODE_PERIOD_MS)
    {
        line_barcode_msg_t msg = line_barcode_step(p_ctx, sensors,
                                                  LINE_BARCODE_PERIOD_MS);
        if (LINE_BARCODE_EVT_BARCODE == msg_event(&msg))
        {
            (*p_count)++;
            *p_letter = msg.data[2] & LINE_BARCODE_LETTER_MASK;
            if (NULL != p_nav)
            {
                (void)line_barcode_nav_update(p_nav, &empty_map, &msg);
            }
        }
    }
}

/* Drives a frame at narrow_ms into the context, then trailing quiet. */
static uint8_t scan_letter(line_barcode_ctx_t *p_ctx, uint8_t letter,
                           bool b_reverse, uint16_t narrow_ms,
                           line_barcode_nav_t *p_nav)
{
    uint16_t runs[LINE_BARCODE_FRAME_RUNS] = {0u};
    uint8_t count = 0u;
    uint8_t decoded = 0xffu;
    build_frame(runs, letter, b_reverse);
    uint16_t quiet_ms = (uint16_t)(10u * narrow_ms + 100u);
    feed_nav(p_ctx, 3u, quiet_ms, p_nav, &count, &decoded);
    for (uint8_t idx = 0u; idx < LINE_BARCODE_FRAME_RUNS; idx++)
    {
        uint16_t width = (uint16_t)((uint32_t)runs[idx] * narrow_ms
                                    / TEST_NARROW_MS);
        uint8_t sensors = (0u == (idx & 1u)) ? 7u : 3u;
        feed_nav(p_ctx, sensors, width, p_nav, &count, &decoded);
    }
    feed_nav(p_ctx, 3u, quiet_ms, p_nav, &count, &decoded);
    return ((1u == count) ? decoded : 0xffu);
}

/* Feeds a crossing line (all black) and counts JUNCTION events. */
static uint8_t cross_line(line_barcode_ctx_t *p_ctx, uint16_t black_ms)
{
    uint8_t junctions = 0u;
    for (uint16_t elapsed = 0u; elapsed < black_ms;
         elapsed += LINE_BARCODE_PERIOD_MS)
    {
        line_barcode_msg_t msg = line_barcode_step(p_ctx, 7u,
                                                  LINE_BARCODE_PERIOD_MS);
        junctions += (LINE_BARCODE_EVT_JUNCTION == msg_event(&msg)) ? 1u : 0u;
    }
    for (uint16_t elapsed = 0u; elapsed < 200u;
         elapsed += LINE_BARCODE_PERIOD_MS)
    {
        line_barcode_msg_t msg = line_barcode_step(p_ctx, 3u,
                                                  LINE_BARCODE_PERIOD_MS);
        junctions += (LINE_BARCODE_EVT_JUNCTION == msg_event(&msg)) ? 1u : 0u;
    }
    return (junctions);
}

/* 18 mm crossing = 6 narrow units; one junction, no INVALID barcode. */
static bool test_side_junction(void)
{
    line_barcode_ctx_t context;
    line_barcode_reset(&context);
    bool b_valid = (0u == scan_letter(&context, 0u, false, TEST_NARROW_MS, NULL));
    b_valid = b_valid && (1u == cross_line(&context, 6u * TEST_NARROW_MS));
    /* Slow scan: wide bars last 600 ms, above the fixed fallback, but the
     * in-frame reference stops them from looking like a crossing.
     */
    line_barcode_reset(&context);
    b_valid = b_valid && (25u == scan_letter(&context, 25u, true, 200u, NULL))
              && (1u == cross_line(&context, 1200u));
    /* A short black patch (a wide bar's length) is not a junction. */
    b_valid = b_valid && (0u == cross_line(&context, 3u * 200u));
    return (b_valid);
}

/* Demo sequence: barcode (either way) -> stored -> junction -> command. */
static bool test_barcode_then_junction(void)
{
    line_barcode_ctx_t context;
    line_barcode_map_t mapping = {{0u}};
    line_barcode_nav_t nav = {0u};
    bool b_valid = line_barcode_map_set(&mapping, 'Z', LINE_BARCODE_CMD_UTURN);
    for (uint8_t direction = 0u; direction < TEST_DIRECTIONS; direction++)
    {
        uint8_t commands = 0u;
        uint8_t command = 0xffu;
        uint8_t letter = 0xffu;
        line_barcode_reset(&context);
        line_barcode_nav_clear(&nav);
        b_valid = b_valid
                  && (25u == scan_letter(&context, 25u, (0u != direction),
                                         TEST_NARROW_MS, &nav))
                  && ((0x80u | 25u) == nav.pending);
        for (uint16_t elapsed = 0u; elapsed < 600u;
             elapsed += LINE_BARCODE_PERIOD_MS)
        {
            uint8_t sensors = (elapsed < 360u) ? 7u : 3u;
            line_barcode_msg_t msg = line_barcode_step(&context, sensors,
                                                      LINE_BARCODE_PERIOD_MS);
            if (line_barcode_nav_update(&nav, &mapping, &msg))
            {
                commands++;
                command = msg.data[0] >> 6u;
                letter = msg.data[2] & LINE_BARCODE_LETTER_MASK;
            }
        }
        b_valid = b_valid && (1u == commands)
                  && (LINE_BARCODE_CMD_UTURN == command) && (25u == letter);
    }
    return (b_valid);
}

static bool test_side_layout(void)
{
    line_barcode_ctx_t context;
    line_barcode_reset(&context);
    (void)line_barcode_step(&context, 3u, LINE_BARCODE_PERIOD_MS);
    (void)line_barcode_step(&context, 3u, LINE_BARCODE_PERIOD_MS);
    bool b_valid = true;
    for (uint8_t idx = 0u; idx < LINE_BARCODE_FRAME_RUNS; idx++)
    {
        uint8_t sensors = (0u == (idx & 1u)) ? 7u : 3u;
        line_barcode_msg_t msg = line_barcode_step(&context, sensors,
                                                  LINE_BARCODE_PERIOD_MS);
        b_valid = b_valid && (2u == ((msg.data[0] >> 3u) & 7u))
                   && (LINE_BARCODE_EVT_JUNCTION != ((msg.data[1] >> 3u) & 7u));
    }
    return (b_valid);
}

static bool test_loss_during_scan(void)
{
    line_barcode_ctx_t context;
    uint8_t count = 0u;
    uint8_t letter = 0u;
    line_barcode_reset(&context);
    (void)feed(&context, 3u, TEST_QUIET_MS, &count, &letter);
    for (uint8_t idx = 0u; idx < 10u; idx++)
    {
        uint8_t sensors = (0u == (idx & 1u)) ? 7u : 3u;
        (void)feed(&context, sensors, g_sample_a[idx], &count, &letter);
    }
    bool b_scanning = (LINE_BARCODE_SCANNING == context.state);
    (void)feed(&context, 0u, 150u, &count, &letter);
    return (b_scanning && (LINE_BARCODE_SEARCH == context.state));
}

static bool test_track_layout(void)
{
    line_barcode_ctx_t context;
    uint8_t count = 0u;
    uint8_t letter = 0u;
    bool b_valid = line_barcode_set_layout(&context, LINE_BARCODE_LAYOUT_TRACK);
    line_barcode_msg_t msg = feed(&context, 7u, 50u, &count, &letter);
    return (b_valid && (LINE_BARCODE_JUNCTION == (msg.data[1] & 7u))
            && !line_barcode_set_layout(&context, 2u));
}

static void test_sample_contract(void)
{
#ifndef LINE_BARCODE_HOST_TEST
    TEST_INIT();
#endif
    TEST_CHECK("S3-U18", test_pdf_samples(), "supplied PDF A/Z width fixtures");
    TEST_CHECK("S3-U19", test_mapping(),
               "mapped and unmapped letters; packed map");
    TEST_CHECK("S3-U20", test_side_layout(),
               "scanner edges never change steering");
    TEST_CHECK("S3-U21", test_loss_during_scan(),
               "line loss during barcode capture");
    TEST_CHECK("S3-U22", test_track_layout(), "three-sensor junction profile");
    TEST_CHECK("S3-U25", test_side_junction(),
               "SIDE cross-junction reported once, wide bars ignored");
    TEST_CHECK("S3-U26", test_barcode_then_junction(),
               "stored letter becomes a command at the next junction");
#ifndef LINE_BARCODE_HOST_TEST
    TEST_SUMMARY("subsystem3 sample contract");
#endif
}

void line_barcode_test(void)
{
    test_sample_contract();
#ifndef LINE_BARCODE_HOST_TEST
    TEST_INIT();
#endif
    line_barcode_ctx_t context;
    uint8_t commands = 0u;
    uint8_t last_command = 0u;
    uint16_t runs[LINE_BARCODE_FRAME_RUNS] = {0u};
    static uint8_t const expected[TEST_SENSOR_COUNT] =
        {6u, 4u, 2u, 3u, 0u, 5u, 1u, 5u};
    bool b_patterns = true;
    bool b_barcodes = true;
    bool b_streams = true;
    for (uint8_t sensors = 0u; sensors < TEST_SENSOR_COUNT; sensors++)
    {
        b_patterns = b_patterns
                     && (expected[sensors] == line_barcode_position(sensors));
    }
    TEST_CHECK("S3-U02", b_patterns, "all eight sensor positions");
    line_barcode_reset(&context);
    (void)feed(&context, 3u, 50u, &commands, &last_command);
    (void)line_barcode_step(&context, 7u, LINE_BARCODE_PERIOD_MS);
    TEST_CHECK("S3-U03", 3u == context.stable, "single-sample noise rejected");
    (void)feed(&context, 0u, 150u, &commands, &last_command);
    TEST_CHECK("S3-U04", LINE_BARCODE_SEARCH == context.state,
               "persistent loss enters bounded search");
    line_barcode_msg_t msg = feed(&context, 3u, 50u, &commands, &last_command);
    TEST_CHECK("S3-U05", LINE_BARCODE_FOLLOW == (msg.data[1] & 7u),
               "line reacquired after search");
    (void)feed(&context, 0u, 2500u, &commands, &last_command);
    TEST_CHECK("S3-U06", LINE_BARCODE_FAULT == context.state,
               "search timeout requests stop");
    for (uint8_t command = 0u; command < TEST_SYMBOLS; command++)
    {
        for (uint8_t direction = 0u; direction < TEST_DIRECTIONS; direction++)
        {
            uint8_t decoded = 0xffu;
            build_frame(runs, command, (0u != direction));
            b_barcodes = b_barcodes
                         && line_barcode_decode(runs, LINE_BARCODE_FRAME_RUNS,
                                                 &decoded)
                         && (command == decoded);
            b_streams = b_streams && test_stream(command, (0u != direction),
                                                 TEST_NARROW_MS);
        }
    }
    bool b_speeds = true;
    static uint16_t const speeds[] = {40u, 100u, 200u};
    for (uint8_t idx = 0u; idx < (sizeof(speeds) / sizeof(speeds[0])); idx++)
    {
        b_speeds = b_speeds && test_stream(0u, false, speeds[idx]);
    }
    TEST_CHECK("S3-U13", b_speeds, "relative widths at multiple scan speeds");
    TEST_CHECK("S3-U07", b_barcodes, "A-Z decode in both directions");
    TEST_CHECK("S3-U08", b_streams, "sampled frames emit exactly one letter");
    build_frame(runs, 0u, false);
    runs[0] = 0u;
    TEST_CHECK("S3-U09", !line_barcode_decode(runs, LINE_BARCODE_FRAME_RUNS,
                                             &last_command),
               "zero-width malformed frame rejected");
    build_frame(runs, 0u, false);
    runs[9] = 1000u;
    TEST_CHECK("S3-U10", !line_barcode_decode(runs, LINE_BARCODE_FRAME_RUNS,
                                             &last_command),
               "invalid inter-character gap rejected");
    msg = line_barcode_step(&context, 2u, 100u);
    TEST_CHECK("S3-U11", 0u != ((msg.data[2] >> LINE_BARCODE_FAULT_SHIFT)
                                & LINE_BARCODE_ERR_TIMING),
               "missed sampling interval invalidates capture");
    msg = line_barcode_pack(7u, 6u, 3u, 5u, 6u, LINE_BARCODE_ERR_MASK);
    TEST_CHECK("S3-U12", (0xf7u == msg.data[0]) && (0x35u == msg.data[1])
                         && (3u == sizeof(msg)) && (0xe0u == msg.data[2]),
               "three-byte wire layout; three faults set at once");
#ifndef LINE_BARCODE_HOST_TEST
    TEST_SUMMARY("subsystem3 line/barcode");
#endif
}

#ifdef LINE_BARCODE_HOST_TEST
int main(void)
{
    line_barcode_test();
    printf("context=%zu bytes, navigation message=%zu bytes\n",
           sizeof(line_barcode_ctx_t), sizeof(line_barcode_msg_t));
    return ((0u == g_failures) ? 0 : 1);
}
#endif
#endif /* APP_TEST */
