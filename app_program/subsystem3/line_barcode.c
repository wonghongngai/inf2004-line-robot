/** @file line_barcode.c
 * @brief Bounded, allocation-free processing of three digital IR inputs.
 */
#include "line_barcode.h"
#include <stddef.h>
#include <string.h>

#define BARCODE_FLAG_BLACK      1u
#define BARCODE_FLAG_CAPTURE    2u
#define BARCODE_FLAG_JUNCTION   4u
#define BARCODE_FLAG_CONFIRMED  8u
#define BARCODE_FLAG_CROSSED    16u
#define BARCODE_ELEMENTS        9u
#define BARCODE_CHAR_STRIDE     10u
#define BARCODE_CHAR_COUNT      3u
#define BARCODE_WIDE_BARS       2u /* of 5 bars, every A-Z and '*' */
#define BARCODE_WIDE_SPACES     1u /* of 4 spaces */
/* Shortest wide must be >= 1.15x the longest narrow, per colour. */
#define BARCODE_SEPARATION_NUM  23u
#define BARCODE_SEPARATION_DEN  20u
#define BARCODE_STAR            0x094u
#define BARCODE_MIN_RUN_MS      LINE_BARCODE_MIN_RUN_MS
#define BARCODE_MIN_QUIET_UNITS 10u
#define BARCODE_MAX_STEP_MS     20u
#define BARCODE_QUIET_MAX_MS    10000u

static uint16_t saturate_add(uint16_t value, uint16_t increment, uint16_t limit)
{
    uint32_t sum = (uint32_t)value + increment;
    return ((sum > limit) ? limit : (uint16_t)sum);
}

line_barcode_msg_t line_barcode_pack(uint8_t sensors, uint8_t position,
                                     uint8_t command, uint8_t state,
                                     uint8_t event, uint8_t fault)
{
    line_barcode_msg_t msg = {{0u, 0u, 0u}};
    msg.data[0] = (uint8_t)((sensors & 7u) | ((position & 7u) << 3u) |
                            ((command & 3u) << 6u));
    msg.data[1] = (uint8_t)((state & 7u) | ((event & 7u) << 3u));
    msg.data[2] =
        (uint8_t)((fault & LINE_BARCODE_ERR_MASK) << LINE_BARCODE_FAULT_SHIFT);
    return (msg);
}

void line_barcode_map_clear(line_barcode_map_t * p_map)
{
    if (NULL != p_map)
    {
        memset(p_map, 0, sizeof(*p_map));
    }
}

bool line_barcode_map_set(line_barcode_map_t * p_map, char letter,
                          uint8_t command)
{
    bool b_valid = (NULL != p_map) && (letter >= 'A') && (letter <= 'Z') &&
                   (command <= LINE_BARCODE_CMD_UTURN);
    if (b_valid)
    {
        uint8_t  bit    = (uint8_t)(letter - 'A') * 3u;
        uint8_t  offset = bit / 8u;
        uint8_t  shift  = bit % 8u;
        uint16_t pair   = p_map->data[offset];
        if ((offset + 1u) < sizeof(p_map->data))
        {
            pair |= (uint16_t)p_map->data[offset + 1u] << 8u;
        }
        pair = (uint16_t)((pair & ~(7u << shift)) | ((4u | command) << shift));
        p_map->data[offset] = (uint8_t)pair;
        if ((offset + 1u) < sizeof(p_map->data))
        {
            p_map->data[offset + 1u] = (uint8_t)(pair >> 8u);
        }
    }
    return (b_valid);
}

/* Returns the 3-bit entry {valid, command[1:0]} for letter 0..25. */
static uint8_t map_lookup(line_barcode_map_t const * p_map, uint8_t letter)
{
    uint8_t  bit    = letter * 3u;
    uint8_t  offset = bit / 8u;
    uint16_t pair   = p_map->data[offset];
    if ((offset + 1u) < sizeof(p_map->data))
    {
        pair |= (uint16_t)p_map->data[offset + 1u] << 8u;
    }
    return ((uint8_t)((pair >> (bit % 8u)) & 7u));
}

bool line_barcode_map_get(line_barcode_map_t const * p_map, uint8_t letter,
                          uint8_t * p_command)
{
    bool b_mapped = false;
    if ((NULL != p_map) && (NULL != p_command) &&
        (letter < LINE_BARCODE_ALPHABET_SIZE))
    {
        uint8_t entry = map_lookup(p_map, letter);
        b_mapped      = (0u != (entry & 4u));
        if (b_mapped)
        {
            *p_command = entry & 3u;
        }
    }
    return (b_mapped);
}

void line_barcode_nav_clear(line_barcode_nav_t * p_nav)
{
    if (NULL != p_nav)
    {
        p_nav->pending = 0u;
    }
}

bool line_barcode_nav_update(line_barcode_nav_t *       p_nav,
                             line_barcode_map_t const * p_map,
                             line_barcode_msg_t *       p_msg)
{
    bool b_command = false;
    if ((NULL == p_nav) || (NULL == p_map) || (NULL == p_msg))
    {
        return (false);
    }
    uint8_t event = (p_msg->data[1] >> 3u) & 7u;
    uint8_t state = p_msg->data[1] & 7u;
    if ((LINE_BARCODE_EVT_BARCODE == event) &&
        ((p_msg->data[2] & LINE_BARCODE_LETTER_MASK) <
         LINE_BARCODE_ALPHABET_SIZE) &&
        ((LINE_BARCODE_FOLLOW == state) || (LINE_BARCODE_SCANNING == state)))
    {
        /* A newer barcode replaces any letter still waiting. */
        p_nav->pending =
            (uint8_t)(0x80u | (p_msg->data[2] & LINE_BARCODE_LETTER_MASK));
    }
    else if ((LINE_BARCODE_EVT_JUNCTION == event) &&
             (0u != (p_nav->pending & 0x80u)))
    {
        uint8_t letter = p_nav->pending & 0x1fu;
        uint8_t entry  = map_lookup(p_map, letter);
        p_nav->pending = 0u;
        b_command      = (0u != (entry & 4u));
        if (b_command)
        {
            p_msg->data[0] =
                (uint8_t)((p_msg->data[0] & 0x3fu) | ((entry & 3u) << 6u));
            p_msg->data[1] = (uint8_t)((p_msg->data[1] & 0xc7u) |
                                       (LINE_BARCODE_EVT_COMMAND << 3u));
            p_msg->data[2] =
                (uint8_t)((p_msg->data[2] & ~LINE_BARCODE_LETTER_MASK) |
                          letter);
        }
    }
    else
    {
        /* Other events neither store nor consume a letter. */
    }
    return (b_command);
}

uint8_t line_barcode_position(uint8_t sensors)
{
    static uint8_t const positions[8] = {6u, 4u, 2u, 3u, 0u, 5u, 1u, 5u};
    return (positions[sensors & LINE_BARCODE_SENSOR_MASK]);
}

void line_barcode_reset(line_barcode_ctx_t * p_ctx)
{
    if (NULL != p_ctx)
    {
        memset(p_ctx, 0, sizeof(*p_ctx));
        p_ctx->last_position = 2u;
        p_ctx->layout        = LINE_BARCODE_LAYOUT_SIDE;
    }
}

bool line_barcode_set_layout(line_barcode_ctx_t * p_ctx, uint8_t layout)
{
    bool b_valid = (NULL != p_ctx) && (layout <= LINE_BARCODE_LAYOUT_SIDE);
    if (b_valid)
    {
        line_barcode_reset(p_ctx);
        p_ctx->layout = layout;
    }
    return (b_valid);
}

/* Two tracking sensors sit just inside opposite edges of the 18 mm line.
 * The third, offset sensor sees the separate barcode and cannot steer.
 * Both track sensors black is centred; it cannot uniquely identify a junction.
 */
static uint8_t track_position(line_barcode_ctx_t const * p_ctx)
{
    static uint8_t const positions[4] = {6u, 4u, 0u, 2u};
    uint8_t              position     = line_barcode_position(p_ctx->stable);
    if (LINE_BARCODE_LAYOUT_SIDE == p_ctx->layout)
    {
        position = positions[p_ctx->stable & 3u];
    }
    return (position);
}

/* Rank-based narrow/wide classification of one nine-element character.
 * Every Code 39 letter A-Z and the '*' marker has exactly two wide bars
 * (of five) and one wide space (of four). So the two longest bars and the
 * longest space are wide; everything else is narrow. No ratio threshold is
 * searched for, which tolerates scan-speed changes within a character that
 * make narrow widths drift (seen on hardware, 19 Nov: narrow bars 71-242 ms
 * in one character). Bars and spaces are ranked separately because the
 * comparator stretches one colour relative to the other.
 *
 * Each colour must still show a minimum wide/narrow separation, so uniform
 * stripes or a corrupted element cannot produce a pattern. The weakest
 * separation in a correctly read hardware frame was 1.20x.
 * Returns the 9-bit pattern (1 = wide, first element in bit 8), or 0. */
static uint16_t decode_character(uint16_t const * p_runs, bool b_reverse)
{
    static uint8_t const wide_wanted[2] = {BARCODE_WIDE_BARS,
                                           BARCODE_WIDE_SPACES};
    uint16_t             is_wide        = 0u; /* bit i set: element i is wide */
    for (unsigned colour = 0u; colour < 2u; ++colour)
    {
        uint8_t  order[5]; /* element indices, longest first */
        unsigned count = 0u;
        for (unsigned i = colour; i < BARCODE_ELEMENTS; i += 2u)
        {
            uint16_t width = p_runs[i];
            if (width < BARCODE_MIN_RUN_MS || width > LINE_BARCODE_RUN_MAX_MS)
                return 0u;
            unsigned j = count++;
            while (j > 0u && p_runs[order[j - 1u]] < width)
            {
                order[j] = order[j - 1u];
                --j;
            }
            order[j] = (uint8_t)i;
        }
        unsigned wide           = wide_wanted[colour];
        uint32_t shortest_wide  = p_runs[order[wide - 1u]];
        uint32_t longest_narrow = p_runs[order[wide]];
        if (shortest_wide * BARCODE_SEPARATION_DEN <
            longest_narrow * BARCODE_SEPARATION_NUM)
            return 0u;
        for (unsigned k = 0u; k < wide; ++k)
            is_wide |= (uint16_t)(1u << order[k]);
    }
    uint16_t pattern = 0u;
    for (unsigned i = 0u; i < BARCODE_ELEMENTS; ++i)
    {
        unsigned source = b_reverse ? BARCODE_ELEMENTS - 1u - i : i;
        pattern = (uint16_t)((pattern << 1u) | ((is_wide >> source) & 1u));
    }
    return pattern;
}

/* Second-longest of a character's four spaces: the longest is its single
 * wide space, so this is its largest narrow space. */
static uint16_t largest_narrow_space(uint16_t const * p_runs)
{
    uint16_t longest = 0u;
    uint16_t second  = 0u;
    for (unsigned i = 1u; i < BARCODE_ELEMENTS; i += 2u)
    {
        uint16_t width = p_runs[i];
        if (width > longest)
        {
            second  = longest;
            longest = width;
        }
        else if (width > second)
        {
            second = width;
        }
    }
    return second;
}

bool line_barcode_decode(uint16_t const * p_runs, uint8_t count,
                         uint8_t * p_letter)
{
    bool b_valid = false;
    if ((NULL == p_runs) || (NULL == p_letter) ||
        (LINE_BARCODE_FRAME_RUNS != count))
    {
        return (false);
    }
    for (uint8_t direction = 0u; direction < 2u; direction++)
    {
        bool     b_reverse = (0u != direction);
        uint16_t first     = decode_character(p_runs, b_reverse);
        uint16_t last      = decode_character(&p_runs[20], b_reverse);
        uint16_t symbol    = decode_character(&p_runs[10], b_reverse);
        /* An inter-character gap is a narrow space. Compare each gap with
         * the largest narrow space of the two characters beside it, not
         * with the shortest run (a 27 ms gap was rejected when a squeezed
         * space was 13 ms) and not with the far end of the frame (scan
         * speed can change across the barcode, 19 Nov). */
        uint16_t narrow_0 = largest_narrow_space(&p_runs[0]);
        uint16_t narrow_1 = largest_narrow_space(&p_runs[10]);
        uint16_t narrow_2 = largest_narrow_space(&p_runs[20]);
        uint32_t gap_0_max =
            2u * (uint32_t)((narrow_0 > narrow_1) ? narrow_0 : narrow_1);
        uint32_t gap_1_max =
            2u * (uint32_t)((narrow_1 > narrow_2) ? narrow_1 : narrow_2);
        bool b_gaps = (p_runs[9] >= BARCODE_MIN_RUN_MS) &&
                      (p_runs[19] >= BARCODE_MIN_RUN_MS) &&
                      (p_runs[9] <= gap_0_max) && (p_runs[19] <= gap_1_max);
        if ((BARCODE_STAR == first) && (BARCODE_STAR == last) && b_gaps)
        {
            static uint16_t const alphabet[LINE_BARCODE_ALPHABET_SIZE] = {
                0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00du,
                0x10cu, 0x04cu, 0x01cu, 0x103u, 0x043u, 0x142u, 0x013u,
                0x112u, 0x052u, 0x007u, 0x106u, 0x046u, 0x016u, 0x181u,
                0x0c1u, 0x1c0u, 0x091u, 0x190u, 0x0d0u};
            for (uint8_t letter = 0u; letter < LINE_BARCODE_ALPHABET_SIZE;
                 letter++)
            {
                if (alphabet[letter] == symbol)
                {
                    *p_letter = letter;
                    b_valid   = true;
                }
            }
        }
    }
    return (b_valid);
}

/* Ten-bit widths are serialized explicitly; no ABI-dependent array padding. */
uint16_t line_barcode_run_width(line_barcode_ctx_t const * p_ctx, uint8_t index)
{
    uint16_t bit  = (uint16_t)index * 10u;
    uint16_t pair = (uint16_t)(p_ctx->runs[bit / 8u] |
                               ((uint16_t)p_ctx->runs[(bit / 8u) + 1u] << 8u));
    return ((pair >> (bit % 8u)) & 0x3ffu);
}

#define read_width line_barcode_run_width

static void write_width(line_barcode_ctx_t * p_ctx, uint8_t index,
                        uint16_t width)
{
    uint16_t bit    = (uint16_t)index * 10u;
    uint8_t  shift  = bit % 8u;
    uint8_t  offset = bit / 8u;
    uint16_t pair   = (uint16_t)(p_ctx->runs[offset] |
                                 ((uint16_t)p_ctx->runs[offset + 1u] << 8u));
    pair =
        (uint16_t)((pair & ~(0x3ffu << shift)) | ((width & 0x3ffu) << shift));
    p_ctx->runs[offset]      = (uint8_t)pair;
    p_ctx->runs[offset + 1u] = (uint8_t)(pair >> 8u);
}

static uint16_t first_narrow(line_barcode_ctx_t const * p_ctx)
{
    uint16_t narrow = LINE_BARCODE_RUN_MAX_MS;
    for (uint8_t idx = 0u; idx < BARCODE_ELEMENTS; idx++)
    {
        uint16_t width = read_width(p_ctx, idx);
        narrow         = (width < narrow) ? width : narrow;
    }
    return (narrow);
}

static void verify_start(line_barcode_ctx_t * p_ctx)
{
    if ((BARCODE_ELEMENTS == p_ctx->run_count) &&
        (0u == (p_ctx->flags & BARCODE_FLAG_CONFIRMED)))
    {
        uint16_t widths[BARCODE_ELEMENTS] = {0u};
        for (uint8_t idx = 0u; idx < BARCODE_ELEMENTS; idx++)
        {
            widths[idx] = read_width(p_ctx, idx);
        }
        bool b_start = (BARCODE_STAR == decode_character(widths, false)) ||
                       (BARCODE_STAR == decode_character(widths, true));
        if (b_start)
        {
            p_ctx->flags |= BARCODE_FLAG_CONFIRMED;
        }
        else
        {
            /* Preserve raw runs; validate only once the frame is complete. */
        }
    }
}

static uint8_t finish_frame(line_barcode_ctx_t * p_ctx, uint8_t * p_letter)
{
    uint16_t widths[LINE_BARCODE_FRAME_RUNS] = {0u};
    for (uint8_t idx = 0u; idx < LINE_BARCODE_FRAME_RUNS; idx++)
    {
        widths[idx] = read_width(p_ctx, idx);
    }
    uint16_t narrow = first_narrow(p_ctx);
    bool     b_valid =
        line_barcode_decode(widths, LINE_BARCODE_FRAME_RUNS, p_letter) &&
        (p_ctx->quiet_ms >= BARCODE_MIN_QUIET_UNITS * narrow);
    if (b_valid)
    {
        /* Scan-speed reference for the following junction. */
        p_ctx->last_narrow = narrow;
    }
    p_ctx->flags &= (uint8_t)~(BARCODE_FLAG_CAPTURE | BARCODE_FLAG_CONFIRMED);
    return (b_valid ? LINE_BARCODE_EVT_BARCODE : LINE_BARCODE_EVT_INVALID);
}

/* The offset left sensor scans the barcode parallel to the guide line.
 * Capture never suppresses line tracking or line-loss detection.
 */
static uint8_t barcode_update(line_barcode_ctx_t * p_ctx, uint16_t elapsed_ms,
                              uint8_t * p_letter)
{
    bool    b_black      = (0u != (p_ctx->stable & LINE_BARCODE_LEFT));
    bool    b_prev_black = (0u != (p_ctx->flags & BARCODE_FLAG_BLACK));
    bool    b_capture    = (0u != (p_ctx->flags & BARCODE_FLAG_CAPTURE));
    uint8_t event        = LINE_BARCODE_EVT_STATUS;
    p_ctx->run_ms =
        saturate_add(p_ctx->run_ms, elapsed_ms, BARCODE_QUIET_MAX_MS + 1u);
    if (b_black != b_prev_black)
    {
        if (b_capture && (p_ctx->run_count < LINE_BARCODE_FRAME_RUNS))
        {
            write_width(p_ctx, p_ctx->run_count, p_ctx->run_ms);
            p_ctx->run_count++;
        }
        if (b_black && !b_capture && (p_ctx->run_ms >= LINE_BARCODE_QUIET_MS))
        {
            p_ctx->flags |= BARCODE_FLAG_CAPTURE;
            p_ctx->quiet_ms  = p_ctx->run_ms;
            p_ctx->run_count = 0u;
        }
        p_ctx->run_ms = 0u;
        p_ctx->flags  = (uint8_t)((p_ctx->flags & ~BARCODE_FLAG_BLACK) |
                                  (b_black ? BARCODE_FLAG_BLACK : 0u));
        verify_start(p_ctx);
    }
    uint16_t quiet_required = BARCODE_MIN_QUIET_UNITS * first_narrow(p_ctx);
    if (!b_black && (LINE_BARCODE_FRAME_RUNS == p_ctx->run_count) &&
        (0u != (p_ctx->flags & BARCODE_FLAG_CAPTURE)) &&
        (p_ctx->run_ms >= quiet_required))
    {
        event = finish_frame(p_ctx, p_letter);
    }
    else if (((p_ctx->run_ms > LINE_BARCODE_RUN_MAX_MS) &&
              (b_black || (LINE_BARCODE_FRAME_RUNS != p_ctx->run_count))) ||
             (b_black && (LINE_BARCODE_FRAME_RUNS == p_ctx->run_count)))
    {
        event = (0u != (p_ctx->flags & BARCODE_FLAG_CAPTURE))
                    ? LINE_BARCODE_EVT_INVALID
                    : LINE_BARCODE_EVT_STATUS;
        p_ctx->flags &=
            (uint8_t)~(BARCODE_FLAG_CAPTURE | BARCODE_FLAG_CONFIRMED);
        p_ctx->run_count = 0u;
    }
    else
    {
        /* Wait for another bounded run or the trailing quiet zone. */
    }
    return (event);
}

/* A crossing line covers all three sensors. The two edge sensors are both
 * black on a straight line too, so only the scanner's run length tells a
 * crossing (about 6 narrow units) from a wide bar (3 units).
 */
static uint8_t side_junction_update(line_barcode_ctx_t * p_ctx)
{
    uint8_t  event     = LINE_BARCODE_EVT_STATUS;
    uint32_t threshold = LINE_BARCODE_JUNCTION_MS;
    uint16_t reference = p_ctx->last_narrow;
    if ((0u != (p_ctx->flags & BARCODE_FLAG_CAPTURE)) &&
        (p_ctx->run_count > 0u))
    {
        /* Inside a frame: the first bar of '*' is narrow either way round. */
        reference = read_width(p_ctx, 0u);
    }
    if (0u != reference)
    {
        threshold = ((uint32_t)reference * LINE_BARCODE_JUNCTION_NUM) /
                    LINE_BARCODE_JUNCTION_DEN;
    }
    if (0u == (p_ctx->stable & LINE_BARCODE_LEFT))
    {
        p_ctx->flags &= (uint8_t)~BARCODE_FLAG_CROSSED;
    }
    else if ((0u == (p_ctx->flags & BARCODE_FLAG_CROSSED)) &&
             (LINE_BARCODE_SENSOR_MASK == p_ctx->stable) &&
             (p_ctx->run_ms >= threshold))
    {
        /* Report once per crossing and drop the partial "barcode". */
        p_ctx->flags |= BARCODE_FLAG_CROSSED;
        p_ctx->flags &=
            (uint8_t)~(BARCODE_FLAG_CAPTURE | BARCODE_FLAG_CONFIRMED);
        p_ctx->run_count = 0u;
        event            = LINE_BARCODE_EVT_JUNCTION;
    }
    else
    {
        /* Scanner black but too short, or this crossing already reported. */
    }
    if ((0u != (p_ctx->flags & BARCODE_FLAG_CROSSED)) &&
        (LINE_BARCODE_SEARCH != p_ctx->state))
    {
        p_ctx->state = LINE_BARCODE_JUNCTION;
    }
    return (event);
}

static uint8_t line_update(line_barcode_ctx_t * p_ctx, uint16_t elapsed_ms,
                           uint8_t position)
{
    uint8_t event = LINE_BARCODE_EVT_STATUS;
    if (6u == position)
    {
        p_ctx->lost_ms =
            saturate_add(p_ctx->lost_ms, elapsed_ms, LINE_BARCODE_LOST_MS);
        if (p_ctx->lost_ms >= LINE_BARCODE_LOST_MS)
        {
            event            = (LINE_BARCODE_SEARCH != p_ctx->state)
                                   ? LINE_BARCODE_EVT_LOST
                                   : LINE_BARCODE_EVT_STATUS;
            p_ctx->state     = LINE_BARCODE_SEARCH;
            p_ctx->search_ms = saturate_add(p_ctx->search_ms, elapsed_ms,
                                            LINE_BARCODE_SEARCH_MS);
        }
    }
    else
    {
        event = (LINE_BARCODE_SEARCH == p_ctx->state) ? LINE_BARCODE_EVT_FOUND
                                                      : LINE_BARCODE_EVT_STATUS;
        p_ctx->lost_ms   = 0u;
        p_ctx->search_ms = 0u;
        p_ctx->state     = LINE_BARCODE_FOLLOW;
        p_ctx->last_position =
            (position < 5u) ? position : p_ctx->last_position;
    }
    bool b_junction = (5u == position);
    if (b_junction)
    {
        p_ctx->state = LINE_BARCODE_JUNCTION;
        if (0u == (p_ctx->flags & BARCODE_FLAG_JUNCTION))
        {
            event = LINE_BARCODE_EVT_JUNCTION;
        }
    }
    p_ctx->flags = (uint8_t)((p_ctx->flags & ~BARCODE_FLAG_JUNCTION) |
                             (b_junction ? BARCODE_FLAG_JUNCTION : 0u));
    return (event);
}

line_barcode_msg_t line_barcode_step(line_barcode_ctx_t * p_ctx,
                                     uint8_t sensors, uint16_t elapsed_ms)
{
    uint8_t letter = 0u;
    uint8_t fault  = LINE_BARCODE_ERR_NONE;
    uint8_t event  = LINE_BARCODE_EVT_STATUS;
    if ((NULL == p_ctx) || (0u == elapsed_ms) ||
        (elapsed_ms > BARCODE_MAX_STEP_MS))
    {
        if (NULL != p_ctx)
        {
            line_barcode_reset(p_ctx);
        }
        return (line_barcode_pack(0u, 6u, 0u, LINE_BARCODE_FAULT,
                                  LINE_BARCODE_EVT_FAULT,
                                  LINE_BARCODE_ERR_TIMING));
    }
    sensors &= LINE_BARCODE_SENSOR_MASK;
    /* A changed input starts a candidate interval. Only subsequent samples
     * with that same level advance its age, so unobserved time before the
     * first sample is never counted as stability. Inputs are independent. */
    for (unsigned bit = 0u; bit < 3u; ++bit)
    {
        uint8_t mask = (uint8_t)(1u << bit);
        if ((sensors ^ p_ctx->candidate) & mask)
        {
            p_ctx->candidate = (p_ctx->candidate & ~mask) | (sensors & mask);
            p_ctx->candidate_ms[bit] = 0u;
        }
        else
        {
            unsigned age = p_ctx->candidate_ms[bit] + elapsed_ms;
            p_ctx->candidate_ms[bit] =
                age < LINE_BARCODE_DEBOUNCE_MS ? age : LINE_BARCODE_DEBOUNCE_MS;
            if (p_ctx->candidate_ms[bit] >= LINE_BARCODE_DEBOUNCE_MS)
                p_ctx->stable = (p_ctx->stable & ~mask) | (sensors & mask);
        }
    }
    uint8_t position       = track_position(p_ctx);
    event                  = line_update(p_ctx, elapsed_ms, position);
    uint8_t barcode_event  = LINE_BARCODE_EVT_STATUS;
    uint8_t junction_event = LINE_BARCODE_EVT_STATUS;
    if (LINE_BARCODE_LAYOUT_SIDE == p_ctx->layout)
    {
        barcode_event  = barcode_update(p_ctx, elapsed_ms, &letter);
        junction_event = side_junction_update(p_ctx);
    }
    /* Slowdown needs to start at capture, before the start symbol is verified. */
    if ((LINE_BARCODE_FOLLOW == p_ctx->state) &&
        (0u != (p_ctx->flags & BARCODE_FLAG_CAPTURE)))
    {
        p_ctx->state = LINE_BARCODE_SCANNING;
    }
    if (LINE_BARCODE_EVT_STATUS != barcode_event)
    {
        event = barcode_event;
    }
    if (LINE_BARCODE_EVT_STATUS != junction_event)
    {
        event = junction_event;
    }
#if LINE_BARCODE_SEARCH_MS > 0
    if (p_ctx->search_ms >= LINE_BARCODE_SEARCH_MS)
    {
        p_ctx->state = LINE_BARCODE_FAULT;
        event        = LINE_BARCODE_EVT_FAULT;
        fault        = LINE_BARCODE_ERR_LINE;
    }
#endif
    /* During short gaps/search, retain the last steering direction. */
    if (6u == position)
    {
        position = p_ctx->last_position;
    }
    line_barcode_msg_t msg = line_barcode_pack(p_ctx->stable, position, 0u,
                                               p_ctx->state, event, fault);
    if ((p_ctx->flags & BARCODE_FLAG_CAPTURE) != 0u)
    {
        msg.data[1] |= LINE_BARCODE_CAPTURE_ACTIVE;
    }
    if (LINE_BARCODE_EVT_BARCODE == event)
    {
        msg.data[2] =
            (uint8_t)((msg.data[2] & ~LINE_BARCODE_LETTER_MASK) | letter);
    }
    return (msg);
}
