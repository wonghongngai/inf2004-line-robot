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
#define TEST_PATTERN_COUNT 8u
#define TEST_NARROW_MS     60u
#define TEST_WIDE_MS       180u
#define TEST_QUIET_MS      700u

/* Sensor patterns used by the tests (true = black). */
static line_barcode_sensors_t const g_on_line = {false, true, true};
static line_barcode_sensors_t const g_all_black = {true, true, true};
static line_barcode_sensors_t const g_no_line = {false, false, false};
static line_barcode_sensors_t const g_centre_only = {false, true, false};

static bool same_sensors(line_barcode_sensors_t first,
                         line_barcode_sensors_t second)
{
    return ((first.b_left == second.b_left)
            && (first.b_centre == second.b_centre)
            && (first.b_right == second.b_right));
}

static void build_frame(uint16_t *p_runs, uint8_t letter, bool b_reverse)
{
    static uint16_t const symbols[TEST_SYMBOLS] =
        {0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00du,
         0x10cu, 0x04cu, 0x01cu, 0x103u, 0x043u, 0x142u, 0x013u,
         0x112u, 0x052u, 0x007u, 0x106u, 0x046u, 0x016u, 0x181u,
         0x0c1u, 0x1c0u, 0x091u, 0x190u, 0x0d0u};
    uint16_t patterns[3] = {0x094u, symbols[letter], 0x094u};
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

/* Even runs are bars (scanner black), odd runs are spaces. */
static line_barcode_sensors_t run_sensors(uint8_t idx)
{
    return ((0u == (idx & 1u)) ? g_all_black : g_on_line);
}

static line_barcode_msg_t feed(line_barcode_ctx_t *p_ctx,
                               line_barcode_sensors_t sensors,
                               uint16_t duration_ms, uint8_t *p_count,
                               char *p_last_letter)
{
    line_barcode_msg_t msg = line_barcode_make_msg(g_no_line,
        LINE_BARCODE_POS_NONE, LINE_BARCODE_FOLLOW, LINE_BARCODE_EVT_STATUS,
        LINE_BARCODE_ERR_NONE);
    for (uint16_t elapsed = 0u; elapsed < duration_ms;
         elapsed += LINE_BARCODE_PERIOD_MS)
    {
        msg = line_barcode_step(p_ctx, sensors, LINE_BARCODE_PERIOD_MS);
        if (LINE_BARCODE_EVT_BARCODE == msg.event)
        {
            (*p_count)++;
            *p_last_letter = msg.letter;
        }
    }
    return (msg);
}

static bool test_stream(uint8_t letter, bool b_reverse, uint16_t narrow_ms)
{
    line_barcode_ctx_t context;
    uint16_t runs[LINE_BARCODE_FRAME_RUNS] = {0u};
    uint8_t count = 0u;
    char last_letter = '\0';
    line_barcode_reset(&context);
    build_frame(runs, letter, b_reverse);
    uint16_t quiet_ms = (uint16_t)(10u * narrow_ms + 100u);
    for (uint8_t idx = 0u; idx < LINE_BARCODE_FRAME_RUNS; idx++)
    {
        runs[idx] = (uint16_t)((uint32_t)runs[idx] * narrow_ms
                               / TEST_NARROW_MS);
    }
    (void)feed(&context, g_on_line, quiet_ms, &count, &last_letter);
    for (uint8_t idx = 0u; idx < LINE_BARCODE_FRAME_RUNS; idx++)
    {
        (void)feed(&context, run_sensors(idx), runs[idx], &count,
                   &last_letter);
    }
    (void)feed(&context, g_on_line, quiet_ms, &count, &last_letter);
    (void)feed(&context, g_on_line, quiet_ms, &count, &last_letter);
    return ((1u == count) && (('A' + letter) == last_letter));
}

static bool test_pdf_samples(void)
{
    char letter = '\0';
    uint16_t reversed[LINE_BARCODE_FRAME_RUNS] = {0u};
    bool b_valid = line_barcode_decode(g_sample_a, LINE_BARCODE_FRAME_RUNS,
                                       &letter) && ('A' == letter);
    b_valid = b_valid && line_barcode_decode(g_sample_z,
        LINE_BARCODE_FRAME_RUNS, &letter) && ('Z' == letter);
    for (uint8_t idx = 0u; idx < LINE_BARCODE_FRAME_RUNS; idx++)
    {
        reversed[idx] = g_sample_z[LINE_BARCODE_FRAME_RUNS - 1u - idx];
    }
    return (b_valid && line_barcode_decode(reversed,
             LINE_BARCODE_FRAME_RUNS, &letter) && ('Z' == letter));
}

static bool test_mapping(void)
{
    line_barcode_map_t mapping;
    line_barcode_nav_t nav;
    line_barcode_msg_t barcode = line_barcode_make_msg(g_on_line,
        LINE_BARCODE_POS_CENTRE, LINE_BARCODE_FOLLOW,
        LINE_BARCODE_EVT_BARCODE, LINE_BARCODE_ERR_NONE);
    line_barcode_msg_t junction = line_barcode_make_msg(g_all_black,
        LINE_BARCODE_POS_CENTRE, LINE_BARCODE_JUNCTION,
        LINE_BARCODE_EVT_JUNCTION, LINE_BARCODE_ERR_NONE);
    line_barcode_msg_t msg = junction;
    line_barcode_map_clear(&mapping);
    line_barcode_nav_clear(&nav);

    /* Unmapped letter: stored, then consumed as a plain junction. */
    barcode.letter = 'Z';
    bool b_valid = !line_barcode_nav_update(&nav, &mapping, &barcode)
                   && (LINE_BARCODE_EVT_BARCODE == barcode.event)
                   && !line_barcode_nav_update(&nav, &mapping, &msg)
                   && (LINE_BARCODE_EVT_JUNCTION == msg.event)
                   && !nav.b_pending;
    for (uint8_t letter = 0u; letter < LINE_BARCODE_ALPHABET_SIZE; letter++)
    {
        b_valid = b_valid && line_barcode_map_set(&mapping,
            (char)('A' + letter), (line_barcode_cmd_t)(letter % 4u));
    }
    for (uint8_t letter = 0u; letter < LINE_BARCODE_ALPHABET_SIZE; letter++)
    {
        barcode.letter = (char)('A' + letter);
        msg = junction;
        b_valid = b_valid && !line_barcode_nav_update(&nav, &mapping, &barcode)
                   && line_barcode_nav_update(&nav, &mapping, &msg)
                   && ((line_barcode_cmd_t)(letter % 4u) == msg.command)
                   && (barcode.letter == msg.letter)
                   && (LINE_BARCODE_EVT_COMMAND == msg.event);
    }
    /* A junction with no letter waiting gives no command. */
    msg = junction;
    b_valid = b_valid && !line_barcode_nav_update(&nav, &mapping, &msg);
    /* Barcodes seen while searching for the line are not stored. */
    barcode.state = LINE_BARCODE_SEARCH;
    (void)line_barcode_nav_update(&nav, &mapping, &barcode);
    b_valid = b_valid && !nav.b_pending;
    line_barcode_map_clear(&mapping);
    return (b_valid && !mapping.entry[0].b_valid
            && !line_barcode_map_set(&mapping, 'a', LINE_BARCODE_CMD_LEFT)
            && !line_barcode_map_set(&mapping, 'Z', LINE_BARCODE_CMD_NONE));
}

/* Steps one sensor pattern for duration_ms; BARCODE events go through the
 * optional nav so the stored letter is the decoder's real output.
 */
static void feed_nav(line_barcode_ctx_t *p_ctx, line_barcode_sensors_t sensors,
                     uint16_t duration_ms, line_barcode_nav_t *p_nav,
                     uint8_t *p_count, char *p_letter)
{
    line_barcode_map_t empty_map;
    line_barcode_map_clear(&empty_map);
    for (uint16_t elapsed = 0u; elapsed < duration_ms;
         elapsed += LINE_BARCODE_PERIOD_MS)
    {
        line_barcode_msg_t msg = line_barcode_step(p_ctx, sensors,
                                                  LINE_BARCODE_PERIOD_MS);
        if (LINE_BARCODE_EVT_BARCODE == msg.event)
        {
            (*p_count)++;
            *p_letter = msg.letter;
            if (NULL != p_nav)
            {
                (void)line_barcode_nav_update(p_nav, &empty_map, &msg);
            }
        }
    }
}

/* Drives a frame at narrow_ms into the context, then trailing quiet. */
static char scan_letter(line_barcode_ctx_t *p_ctx, uint8_t letter,
                        bool b_reverse, uint16_t narrow_ms,
                        line_barcode_nav_t *p_nav)
{
    uint16_t runs[LINE_BARCODE_FRAME_RUNS] = {0u};
    uint8_t count = 0u;
    char decoded = '\0';
    build_frame(runs, letter, b_reverse);
    uint16_t quiet_ms = (uint16_t)(10u * narrow_ms + 100u);
    feed_nav(p_ctx, g_on_line, quiet_ms, p_nav, &count, &decoded);
    for (uint8_t idx = 0u; idx < LINE_BARCODE_FRAME_RUNS; idx++)
    {
        uint16_t width = (uint16_t)((uint32_t)runs[idx] * narrow_ms
                                    / TEST_NARROW_MS);
        feed_nav(p_ctx, run_sensors(idx), width, p_nav, &count, &decoded);
    }
    feed_nav(p_ctx, g_on_line, quiet_ms, p_nav, &count, &decoded);
    return ((1u == count) ? decoded : '\0');
}

/* Feeds a crossing line (all black) and counts JUNCTION events. */
static uint8_t cross_line(line_barcode_ctx_t *p_ctx, uint16_t black_ms)
{
    uint8_t junctions = 0u;
    for (uint16_t elapsed = 0u; elapsed < black_ms;
         elapsed += LINE_BARCODE_PERIOD_MS)
    {
        line_barcode_msg_t msg = line_barcode_step(p_ctx, g_all_black,
                                                  LINE_BARCODE_PERIOD_MS);
        junctions += (LINE_BARCODE_EVT_JUNCTION == msg.event) ? 1u : 0u;
    }
    for (uint16_t elapsed = 0u; elapsed < 200u;
         elapsed += LINE_BARCODE_PERIOD_MS)
    {
        line_barcode_msg_t msg = line_barcode_step(p_ctx, g_on_line,
                                                  LINE_BARCODE_PERIOD_MS);
        junctions += (LINE_BARCODE_EVT_JUNCTION == msg.event) ? 1u : 0u;
    }
    return (junctions);
}

/* 18 mm crossing = 6 narrow units; one junction, no INVALID barcode. */
static bool test_side_junction(void)
{
    line_barcode_ctx_t context;
    line_barcode_reset(&context);
    bool b_valid = ('A' == scan_letter(&context, 0u, false, TEST_NARROW_MS,
                                       NULL));
    b_valid = b_valid && (1u == cross_line(&context, 6u * TEST_NARROW_MS));
    /* Slow scan: wide bars last 600 ms, above the fixed fallback, but the
     * in-frame reference stops them from looking like a crossing.
     */
    line_barcode_reset(&context);
    b_valid = b_valid && ('Z' == scan_letter(&context, 25u, true, 200u, NULL))
              && (1u == cross_line(&context, 1200u));
    /* A short black patch (a wide bar's length) is not a junction. */
    b_valid = b_valid && (0u == cross_line(&context, 3u * 200u));
    return (b_valid);
}

/* Demo sequence: barcode (either way) -> stored -> junction -> command. */
static bool test_barcode_then_junction(void)
{
    line_barcode_ctx_t context;
    line_barcode_map_t mapping;
    line_barcode_nav_t nav;
    line_barcode_map_clear(&mapping);
    bool b_valid = line_barcode_map_set(&mapping, 'Z', LINE_BARCODE_CMD_UTURN);
    for (uint8_t direction = 0u; direction < TEST_DIRECTIONS; direction++)
    {
        uint8_t commands = 0u;
        line_barcode_cmd_t command = LINE_BARCODE_CMD_NONE;
        char letter = '\0';
        line_barcode_reset(&context);
        line_barcode_nav_clear(&nav);
        b_valid = b_valid
                  && ('Z' == scan_letter(&context, 25u, (0u != direction),
                                         TEST_NARROW_MS, &nav))
                  && nav.b_pending && ('Z' == nav.letter);
        for (uint16_t elapsed = 0u; elapsed < 600u;
             elapsed += LINE_BARCODE_PERIOD_MS)
        {
            line_barcode_sensors_t sensors = (elapsed < 360u)
                                             ? g_all_black : g_on_line;
            line_barcode_msg_t msg = line_barcode_step(&context, sensors,
                                                      LINE_BARCODE_PERIOD_MS);
            if (line_barcode_nav_update(&nav, &mapping, &msg))
            {
                commands++;
                command = msg.command;
                letter = msg.letter;
            }
        }
        b_valid = b_valid && (1u == commands)
                  && (LINE_BARCODE_CMD_UTURN == command) && ('Z' == letter);
    }
    return (b_valid);
}

static bool test_side_layout(void)
{
    line_barcode_ctx_t context;
    line_barcode_reset(&context);
    (void)line_barcode_step(&context, g_on_line, LINE_BARCODE_PERIOD_MS);
    (void)line_barcode_step(&context, g_on_line, LINE_BARCODE_PERIOD_MS);
    bool b_valid = true;
    for (uint8_t idx = 0u; idx < LINE_BARCODE_FRAME_RUNS; idx++)
    {
        line_barcode_msg_t msg = line_barcode_step(&context, run_sensors(idx),
                                                  LINE_BARCODE_PERIOD_MS);
        b_valid = b_valid && (LINE_BARCODE_POS_CENTRE == msg.position)
                   && (LINE_BARCODE_EVT_JUNCTION != msg.event);
    }
    return (b_valid);
}

static bool test_loss_during_scan(void)
{
    line_barcode_ctx_t context;
    uint8_t count = 0u;
    char letter = '\0';
    line_barcode_reset(&context);
    (void)feed(&context, g_on_line, TEST_QUIET_MS, &count, &letter);
    for (uint8_t idx = 0u; idx < 10u; idx++)
    {
        (void)feed(&context, run_sensors(idx), g_sample_a[idx], &count,
                   &letter);
    }
    bool b_scanning = (LINE_BARCODE_SCANNING == context.state);
    (void)feed(&context, g_no_line, 150u, &count, &letter);
    return (b_scanning && (LINE_BARCODE_SEARCH == context.state));
}

static bool test_track_layout(void)
{
    line_barcode_ctx_t context;
    uint8_t count = 0u;
    char letter = '\0';
    bool b_valid = line_barcode_set_layout(&context, LINE_BARCODE_LAYOUT_TRACK);
    line_barcode_msg_t msg = feed(&context, g_all_black, 50u, &count,
                                  &letter);
    return (b_valid && (LINE_BARCODE_JUNCTION == msg.state)
            && !line_barcode_set_layout(&context,
                                        (line_barcode_layout_t)2));
}

static void test_sample_contract(void)
{
#ifndef LINE_BARCODE_HOST_TEST
    TEST_INIT();
#endif
    TEST_CHECK("S3-U18", test_pdf_samples(), "supplied PDF A/Z width fixtures");
    TEST_CHECK("S3-U19", test_mapping(), "mapped and unmapped letters");
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
    uint8_t count = 0u;
    char last_letter = '\0';
    uint16_t runs[LINE_BARCODE_FRAME_RUNS] = {0u};
    /* Patterns 0..7 as left/centre/right bits, with expected position. */
    static line_barcode_position_t const expected[TEST_PATTERN_COUNT] =
    {
        LINE_BARCODE_POS_NONE, LINE_BARCODE_POS_FAR_RIGHT,
        LINE_BARCODE_POS_CENTRE, LINE_BARCODE_POS_RIGHT,
        LINE_BARCODE_POS_FAR_LEFT, LINE_BARCODE_POS_JUNCTION,
        LINE_BARCODE_POS_LEFT, LINE_BARCODE_POS_JUNCTION
    };
    bool b_patterns = true;
    bool b_barcodes = true;
    bool b_streams = true;
    for (uint8_t pattern = 0u; pattern < TEST_PATTERN_COUNT; pattern++)
    {
        line_barcode_sensors_t sensors = {(pattern & 4u) != 0u,
                                          (pattern & 2u) != 0u,
                                          (pattern & 1u) != 0u};
        b_patterns = b_patterns
                     && (expected[pattern] == line_barcode_position(sensors));
    }
    TEST_CHECK("S3-U02", b_patterns, "all eight sensor positions");
    line_barcode_reset(&context);
    (void)feed(&context, g_on_line, 50u, &count, &last_letter);
    (void)line_barcode_step(&context, g_all_black, LINE_BARCODE_PERIOD_MS);
    TEST_CHECK("S3-U03", same_sensors(g_on_line, context.stable),
               "single-sample noise rejected");
    (void)feed(&context, g_no_line, 150u, &count, &last_letter);
    TEST_CHECK("S3-U04", LINE_BARCODE_SEARCH == context.state,
               "persistent loss enters bounded search");
    line_barcode_msg_t msg = feed(&context, g_on_line, 50u, &count,
                                  &last_letter);
    TEST_CHECK("S3-U05", LINE_BARCODE_FOLLOW == msg.state,
               "line reacquired after search");
    (void)feed(&context, g_no_line, 2500u, &count, &last_letter);
    TEST_CHECK("S3-U06", LINE_BARCODE_FAULT == context.state,
               "search timeout requests stop");
    for (uint8_t letter = 0u; letter < TEST_SYMBOLS; letter++)
    {
        for (uint8_t direction = 0u; direction < TEST_DIRECTIONS; direction++)
        {
            char decoded = '\0';
            build_frame(runs, letter, (0u != direction));
            b_barcodes = b_barcodes
                         && line_barcode_decode(runs, LINE_BARCODE_FRAME_RUNS,
                                                 &decoded)
                         && (('A' + letter) == decoded);
            b_streams = b_streams && test_stream(letter, (0u != direction),
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
                                             &last_letter),
               "zero-width malformed frame rejected");
    build_frame(runs, 0u, false);
    runs[9] = 1000u;
    TEST_CHECK("S3-U10", !line_barcode_decode(runs, LINE_BARCODE_FRAME_RUNS,
                                             &last_letter),
               "invalid inter-character gap rejected");
    msg = line_barcode_step(&context, g_centre_only, 100u);
    TEST_CHECK("S3-U11", LINE_BARCODE_ERR_TIMING == msg.fault,
               "missed sampling interval invalidates capture");
    msg = line_barcode_make_msg(g_all_black, LINE_BARCODE_POS_NONE,
                                LINE_BARCODE_FAULT, LINE_BARCODE_EVT_FAULT,
                                LINE_BARCODE_ERR_TIMING);
    TEST_CHECK("S3-U12", same_sensors(g_all_black, msg.sensors)
                         && (LINE_BARCODE_POS_NONE == msg.position)
                         && (LINE_BARCODE_FAULT == msg.state)
                         && (LINE_BARCODE_EVT_FAULT == msg.event)
                         && (LINE_BARCODE_ERR_TIMING == msg.fault)
                         && (LINE_BARCODE_CMD_NONE == msg.command)
                         && ('\0' == msg.letter),
               "message fields hold the values given");
#ifndef LINE_BARCODE_HOST_TEST
    TEST_SUMMARY("subsystem3 line/barcode");
#endif
}

#ifdef LINE_BARCODE_HOST_TEST
int main(void)
{
    line_barcode_test();
    printf("context=%zu bytes, navigation message=%zu bytes, "
           "letter map=%zu bytes, nav=%zu bytes\n",
           sizeof(line_barcode_ctx_t), sizeof(line_barcode_msg_t),
           sizeof(line_barcode_map_t), sizeof(line_barcode_nav_t));
    return ((0u == g_failures) ? 0 : 1);
}
#endif
#endif /* APP_TEST */

/*** end of file ***/
