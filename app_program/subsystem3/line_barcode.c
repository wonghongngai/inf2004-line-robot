/** @file line_barcode.c
 * @brief Bounded, allocation-free processing of three digital IR inputs.
 *
 * BEFORE version: plain fields, no bit packing (see line_barcode.h).
 */
#include "line_barcode.h"
#include <stddef.h>
#include <string.h>

#define BARCODE_ELEMENTS         9u
#define BARCODE_WIDE_COUNT       3u
#define BARCODE_STAR             0x094u
#define BARCODE_MIN_RUN_MS       40u
#define BARCODE_MIN_QUIET_UNITS  10u
#define BARCODE_MAX_STEP_MS      20u
#define BARCODE_QUIET_MAX_MS     10000u

static uint32_t saturate_add(uint32_t value, uint32_t increment,
                             uint32_t limit)
{
    uint32_t sum = value + increment;
    return ((sum > limit) ? limit : sum);
}

static bool all_black(line_barcode_sensors_t sensors)
{
    return (sensors.b_left && sensors.b_centre && sensors.b_right);
}

line_barcode_msg_t line_barcode_make_msg(line_barcode_sensors_t sensors,
                                        line_barcode_position_t position,
                                        line_barcode_state_t state,
                                        line_barcode_event_t event,
                                        line_barcode_fault_t fault)
{
    line_barcode_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.sensors = sensors;
    msg.position = position;
    msg.state = state;
    msg.event = event;
    msg.command = LINE_BARCODE_CMD_NONE;
    msg.fault = fault;
    msg.letter = '\0';
    return (msg);
}

void line_barcode_map_clear(line_barcode_map_t *p_map)
{
    if (NULL != p_map)
    {
        for (uint8_t idx = 0u; idx < LINE_BARCODE_ALPHABET_SIZE; idx++)
        {
            p_map->entry[idx].b_valid = false;
            p_map->entry[idx].command = LINE_BARCODE_CMD_NONE;
        }
    }
}

bool line_barcode_map_set(line_barcode_map_t *p_map, char letter,
                           line_barcode_cmd_t command)
{
    bool b_valid = (NULL != p_map) && (letter >= 'A') && (letter <= 'Z')
                   && (command <= LINE_BARCODE_CMD_UTURN);
    if (b_valid)
    {
        p_map->entry[letter - 'A'].b_valid = true;
        p_map->entry[letter - 'A'].command = command;
    }
    return (b_valid);
}

void line_barcode_nav_clear(line_barcode_nav_t *p_nav)
{
    if (NULL != p_nav)
    {
        p_nav->b_pending = false;
        p_nav->letter = '\0';
    }
}

bool line_barcode_nav_update(line_barcode_nav_t *p_nav,
                             line_barcode_map_t const *p_map,
                             line_barcode_msg_t *p_msg)
{
    bool b_command = false;
    if ((NULL == p_nav) || (NULL == p_map) || (NULL == p_msg))
    {
        return (false);
    }
    if ((LINE_BARCODE_EVT_BARCODE == p_msg->event)
        && (p_msg->letter >= 'A') && (p_msg->letter <= 'Z')
        && ((LINE_BARCODE_FOLLOW == p_msg->state)
            || (LINE_BARCODE_SCANNING == p_msg->state)))
    {
        /* A newer barcode replaces any letter still waiting. */
        p_nav->b_pending = true;
        p_nav->letter = p_msg->letter;
    }
    else if ((LINE_BARCODE_EVT_JUNCTION == p_msg->event) && p_nav->b_pending)
    {
        line_barcode_map_entry_t entry = p_map->entry[p_nav->letter - 'A'];
        b_command = entry.b_valid;
        if (b_command)
        {
            p_msg->event = LINE_BARCODE_EVT_COMMAND;
            p_msg->command = entry.command;
            p_msg->letter = p_nav->letter;
        }
        line_barcode_nav_clear(p_nav);
    }
    else
    {
        /* Other events neither store nor consume a letter. */
    }
    return (b_command);
}

/* Three-sensor TRACK profile. */
line_barcode_position_t line_barcode_position(line_barcode_sensors_t sensors)
{
    line_barcode_position_t position = LINE_BARCODE_POS_NONE;
    bool b_left = sensors.b_left;
    bool b_centre = sensors.b_centre;
    bool b_right = sensors.b_right;

    if (b_left && b_right)
    {
        position = LINE_BARCODE_POS_JUNCTION;
    }
    else if (b_left && b_centre)
    {
        position = LINE_BARCODE_POS_LEFT;
    }
    else if (b_centre && b_right)
    {
        position = LINE_BARCODE_POS_RIGHT;
    }
    else if (b_left)
    {
        position = LINE_BARCODE_POS_FAR_LEFT;
    }
    else if (b_right)
    {
        position = LINE_BARCODE_POS_FAR_RIGHT;
    }
    else if (b_centre)
    {
        position = LINE_BARCODE_POS_CENTRE;
    }
    else
    {
        position = LINE_BARCODE_POS_NONE;
    }
    return (position);
}

void line_barcode_reset(line_barcode_ctx_t *p_ctx)
{
    if (NULL != p_ctx)
    {
        memset(p_ctx, 0, sizeof(*p_ctx));
        p_ctx->layout = LINE_BARCODE_LAYOUT_SIDE;
        p_ctx->state = LINE_BARCODE_FOLLOW;
        p_ctx->last_position = LINE_BARCODE_POS_CENTRE;
    }
}

bool line_barcode_set_layout(line_barcode_ctx_t *p_ctx,
                             line_barcode_layout_t layout)
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
 */
static line_barcode_position_t track_position(line_barcode_ctx_t const *p_ctx)
{
    line_barcode_position_t position = line_barcode_position(p_ctx->stable);
    if (LINE_BARCODE_LAYOUT_SIDE == p_ctx->layout)
    {
        bool b_left_edge = p_ctx->stable.b_centre;
        bool b_right_edge = p_ctx->stable.b_right;
        if (b_left_edge && b_right_edge)
        {
            position = LINE_BARCODE_POS_CENTRE;
        }
        else if (b_left_edge)
        {
            position = LINE_BARCODE_POS_FAR_LEFT;
        }
        else if (b_right_edge)
        {
            position = LINE_BARCODE_POS_FAR_RIGHT;
        }
        else
        {
            position = LINE_BARCODE_POS_NONE;
        }
    }
    return (position);
}

/* Classify each character locally to tolerate gradual changes in speed.
 * Reject overlaps, weak contrast, outliers and anything but three wide runs.
 */
static uint16_t decode_character(uint16_t const *p_runs, bool b_reverse)
{
    uint16_t minimum = LINE_BARCODE_RUN_MAX_MS;
    uint16_t maximum = 0u;
    uint16_t pattern = 0u;
    uint8_t wide_count = 0u;
    bool b_valid = true;

    for (uint8_t idx = 0u; idx < BARCODE_ELEMENTS; idx++)
    {
        uint16_t width = p_runs[idx];
        minimum = (width < minimum) ? width : minimum;
        maximum = (width > maximum) ? width : maximum;
    }
    if ((minimum < BARCODE_MIN_RUN_MS)
        || (maximum > LINE_BARCODE_RUN_MAX_MS)
        || ((uint32_t)maximum * 2u < (uint32_t)minimum * 3u)
        || ((uint32_t)maximum > (uint32_t)minimum * 4u))
    {
        b_valid = false;
    }
    for (uint8_t idx = 0u; idx < BARCODE_ELEMENTS; idx++)
    {
        uint8_t source = b_reverse ? (BARCODE_ELEMENTS - 1u - idx) : idx;
        bool b_wide = ((uint32_t)p_runs[source] * 2u
                       > (uint32_t)minimum + maximum);
        pattern = (uint16_t)(pattern << 1u);
        if (b_wide)
        {
            pattern |= 1u;
            wide_count++;
        }
    }
    return ((b_valid && (BARCODE_WIDE_COUNT == wide_count)) ? pattern : 0u);
}

bool line_barcode_decode(uint16_t const *p_runs, uint8_t count,
                         char *p_letter)
{
    /* Code 39 element patterns (1 = wide), A..Z. */
    static uint16_t const alphabet[LINE_BARCODE_ALPHABET_SIZE] =
    {
        0x109u, 0x049u, 0x148u, 0x019u, 0x118u, 0x058u, 0x00du,
        0x10cu, 0x04cu, 0x01cu, 0x103u, 0x043u, 0x142u, 0x013u,
        0x112u, 0x052u, 0x007u, 0x106u, 0x046u, 0x016u, 0x181u,
        0x0c1u, 0x1c0u, 0x091u, 0x190u, 0x0d0u
    };
    bool b_valid = false;
    if ((NULL == p_runs) || (NULL == p_letter)
        || (LINE_BARCODE_FRAME_RUNS != count))
    {
        return (false);
    }
    for (uint8_t direction = 0u; direction < 2u; direction++)
    {
        bool b_reverse = (0u != direction);
        uint16_t first = decode_character(p_runs, b_reverse);
        uint16_t last = decode_character(&p_runs[20], b_reverse);
        uint16_t symbol = decode_character(&p_runs[10], b_reverse);
        uint16_t gap_min = p_runs[0];
        for (uint8_t idx = 0u; idx < BARCODE_ELEMENTS; idx++)
        {
            gap_min = (p_runs[idx] < gap_min) ? p_runs[idx] : gap_min;
        }
        bool b_gaps = (p_runs[9] >= BARCODE_MIN_RUN_MS)
                       && (p_runs[19] >= BARCODE_MIN_RUN_MS)
                       && (p_runs[9] <= (2u * gap_min))
                       && (p_runs[19] <= (2u * gap_min));
        if ((BARCODE_STAR == first) && (BARCODE_STAR == last) && b_gaps)
        {
            for (uint8_t letter = 0u; letter < LINE_BARCODE_ALPHABET_SIZE;
                 letter++)
            {
                if (alphabet[letter] == symbol)
                {
                    *p_letter = (char)('A' + letter);
                    b_valid = true;
                }
            }
        }
    }
    return (b_valid);
}

static uint16_t first_narrow(line_barcode_ctx_t const *p_ctx)
{
    uint16_t narrow = LINE_BARCODE_RUN_MAX_MS;
    for (uint8_t idx = 0u; idx < BARCODE_ELEMENTS; idx++)
    {
        uint16_t width = p_ctx->runs[idx];
        narrow = (width < narrow) ? width : narrow;
    }
    return (narrow);
}

static void verify_start(line_barcode_ctx_t *p_ctx)
{
    if ((BARCODE_ELEMENTS == p_ctx->run_count) && !p_ctx->b_confirmed)
    {
        bool b_start = (BARCODE_STAR == decode_character(p_ctx->runs, false))
                       || (BARCODE_STAR == decode_character(p_ctx->runs,
                                                            true));
        if (b_start)
        {
            p_ctx->b_confirmed = true;
        }
        else
        {
            p_ctx->b_capture = false;
        }
    }
}

static line_barcode_event_t finish_frame(line_barcode_ctx_t *p_ctx,
                                         char *p_letter)
{
    uint16_t narrow = first_narrow(p_ctx);
    bool b_valid = line_barcode_decode(p_ctx->runs, LINE_BARCODE_FRAME_RUNS,
                                       p_letter)
                  && (p_ctx->quiet_ms
                      >= (uint32_t)BARCODE_MIN_QUIET_UNITS * narrow);
    if (b_valid)
    {
        /* Scan-speed reference for the following junction. */
        p_ctx->last_narrow = narrow;
    }
    p_ctx->b_capture = false;
    p_ctx->b_confirmed = false;
    return (b_valid ? LINE_BARCODE_EVT_BARCODE : LINE_BARCODE_EVT_INVALID);
}

/* The offset left sensor scans the barcode parallel to the guide line.
 * Capture never suppresses line tracking or line-loss detection.
 */
static line_barcode_event_t barcode_update(line_barcode_ctx_t *p_ctx,
                                           uint16_t elapsed_ms,
                                           char *p_letter)
{
    bool b_black = p_ctx->stable.b_left;
    line_barcode_event_t event = LINE_BARCODE_EVT_STATUS;
    p_ctx->run_ms = saturate_add(p_ctx->run_ms, elapsed_ms,
                                 BARCODE_QUIET_MAX_MS + 1u);
    if (b_black != p_ctx->b_black)
    {
        if (p_ctx->b_capture && (p_ctx->run_count < LINE_BARCODE_FRAME_RUNS))
        {
            p_ctx->runs[p_ctx->run_count] = (uint16_t)p_ctx->run_ms;
            p_ctx->run_count++;
        }
        if (b_black && !p_ctx->b_capture
            && (p_ctx->run_ms >= LINE_BARCODE_QUIET_MS))
        {
            p_ctx->b_capture = true;
            p_ctx->quiet_ms = p_ctx->run_ms;
            p_ctx->run_count = 0u;
        }
        p_ctx->run_ms = 0u;
        p_ctx->b_black = b_black;
        verify_start(p_ctx);
    }
    uint32_t quiet_required = (uint32_t)BARCODE_MIN_QUIET_UNITS
                              * first_narrow(p_ctx);
    if (!b_black && (LINE_BARCODE_FRAME_RUNS == p_ctx->run_count)
        && p_ctx->b_capture && (p_ctx->run_ms >= quiet_required))
    {
        event = finish_frame(p_ctx, p_letter);
    }
    else if (((p_ctx->run_ms > LINE_BARCODE_RUN_MAX_MS)
              && (b_black || (LINE_BARCODE_FRAME_RUNS != p_ctx->run_count)))
             || (b_black && (LINE_BARCODE_FRAME_RUNS == p_ctx->run_count)))
    {
        event = p_ctx->b_confirmed ? LINE_BARCODE_EVT_INVALID
                                   : LINE_BARCODE_EVT_STATUS;
        p_ctx->b_capture = false;
        p_ctx->b_confirmed = false;
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
static line_barcode_event_t side_junction_update(line_barcode_ctx_t *p_ctx)
{
    line_barcode_event_t event = LINE_BARCODE_EVT_STATUS;
    uint32_t threshold = LINE_BARCODE_JUNCTION_MS;
    uint16_t reference = p_ctx->last_narrow;
    if (p_ctx->b_capture && (p_ctx->run_count > 0u))
    {
        /* Inside a frame: the first bar of '*' is narrow either way round. */
        reference = p_ctx->runs[0];
    }
    if (0u != reference)
    {
        threshold = ((uint32_t)reference * LINE_BARCODE_JUNCTION_NUM)
                    / LINE_BARCODE_JUNCTION_DEN;
    }
    if (!p_ctx->stable.b_left)
    {
        p_ctx->b_crossed = false;
    }
    else if (!p_ctx->b_crossed && all_black(p_ctx->stable)
             && (p_ctx->run_ms >= threshold))
    {
        /* Report once per crossing and drop the partial "barcode". */
        p_ctx->b_crossed = true;
        p_ctx->b_capture = false;
        p_ctx->b_confirmed = false;
        p_ctx->run_count = 0u;
        event = LINE_BARCODE_EVT_JUNCTION;
    }
    else
    {
        /* Scanner black but too short, or this crossing already reported. */
    }
    if (p_ctx->b_crossed && (LINE_BARCODE_SEARCH != p_ctx->state))
    {
        p_ctx->state = LINE_BARCODE_JUNCTION;
    }
    return (event);
}

static line_barcode_event_t line_update(line_barcode_ctx_t *p_ctx,
                                        uint16_t elapsed_ms,
                                        line_barcode_position_t position)
{
    line_barcode_event_t event = LINE_BARCODE_EVT_STATUS;
    if (LINE_BARCODE_POS_NONE == position)
    {
        p_ctx->lost_ms = saturate_add(p_ctx->lost_ms, elapsed_ms,
                                      LINE_BARCODE_LOST_MS);
        if (p_ctx->lost_ms >= LINE_BARCODE_LOST_MS)
        {
            event = (LINE_BARCODE_SEARCH != p_ctx->state)
                    ? LINE_BARCODE_EVT_LOST : LINE_BARCODE_EVT_STATUS;
            p_ctx->state = LINE_BARCODE_SEARCH;
            p_ctx->search_ms = saturate_add(p_ctx->search_ms, elapsed_ms,
                                            LINE_BARCODE_SEARCH_MS);
        }
    }
    else
    {
        event = (LINE_BARCODE_SEARCH == p_ctx->state)
                ? LINE_BARCODE_EVT_FOUND : LINE_BARCODE_EVT_STATUS;
        p_ctx->lost_ms = 0u;
        p_ctx->search_ms = 0u;
        p_ctx->state = LINE_BARCODE_FOLLOW;
        if (position < LINE_BARCODE_POS_JUNCTION)
        {
            p_ctx->last_position = position;
        }
    }
    bool b_junction = (LINE_BARCODE_POS_JUNCTION == position);
    if (b_junction)
    {
        p_ctx->state = LINE_BARCODE_JUNCTION;
        if (!p_ctx->b_junction)
        {
            event = LINE_BARCODE_EVT_JUNCTION;
        }
    }
    p_ctx->b_junction = b_junction;
    return (event);
}

/* Each sensor must read the same on two samples in a row to change. */
static bool debounce(bool b_sample, bool *p_candidate, bool b_stable)
{
    bool b_result = (b_sample == *p_candidate) ? b_sample : b_stable;
    *p_candidate = b_sample;
    return (b_result);
}

line_barcode_msg_t line_barcode_step(line_barcode_ctx_t *p_ctx,
                                    line_barcode_sensors_t sensors,
                                    uint16_t elapsed_ms)
{
    static line_barcode_sensors_t const no_sensors = {false, false, false};
    char letter = '\0';
    line_barcode_fault_t fault = LINE_BARCODE_ERR_NONE;
    line_barcode_event_t event = LINE_BARCODE_EVT_STATUS;
    if ((NULL == p_ctx) || (0u == elapsed_ms)
        || (elapsed_ms > BARCODE_MAX_STEP_MS))
    {
        if (NULL != p_ctx)
        {
            line_barcode_reset(p_ctx);
        }
        return (line_barcode_make_msg(no_sensors, LINE_BARCODE_POS_NONE,
                                      LINE_BARCODE_FAULT,
                                      LINE_BARCODE_EVT_FAULT,
                                      LINE_BARCODE_ERR_TIMING));
    }
    /* Debounce each sensor independently, so scanner edges cannot delay
     * line sensing.
     */
    p_ctx->stable.b_left = debounce(sensors.b_left, &p_ctx->candidate.b_left,
                                    p_ctx->stable.b_left);
    p_ctx->stable.b_centre = debounce(sensors.b_centre,
                                      &p_ctx->candidate.b_centre,
                                      p_ctx->stable.b_centre);
    p_ctx->stable.b_right = debounce(sensors.b_right,
                                     &p_ctx->candidate.b_right,
                                     p_ctx->stable.b_right);
    line_barcode_position_t position = track_position(p_ctx);
    event = line_update(p_ctx, elapsed_ms, position);
    line_barcode_event_t barcode_event = LINE_BARCODE_EVT_STATUS;
    line_barcode_event_t junction_event = LINE_BARCODE_EVT_STATUS;
    if (LINE_BARCODE_LAYOUT_SIDE == p_ctx->layout)
    {
        barcode_event = barcode_update(p_ctx, elapsed_ms, &letter);
        junction_event = side_junction_update(p_ctx);
    }
    if ((LINE_BARCODE_FOLLOW == p_ctx->state) && p_ctx->b_confirmed)
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
    if (p_ctx->search_ms >= LINE_BARCODE_SEARCH_MS)
    {
        p_ctx->state = LINE_BARCODE_FAULT;
        event = LINE_BARCODE_EVT_FAULT;
        fault = LINE_BARCODE_ERR_LINE;
    }
    /* During short gaps/search, retain the last steering direction. */
    if (LINE_BARCODE_POS_NONE == position)
    {
        position = p_ctx->last_position;
    }
    line_barcode_msg_t msg = line_barcode_make_msg(p_ctx->stable, position,
                                                  p_ctx->state, event, fault);
    if (LINE_BARCODE_EVT_BARCODE == event)
    {
        msg.letter = letter;
    }
    return (msg);
}

/*** end of file ***/
