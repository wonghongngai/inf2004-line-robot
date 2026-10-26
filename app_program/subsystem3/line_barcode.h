/** @file line_barcode.h
 * @brief Portable line processing and Code 39 A-Z decoding.
 *
 * BEFORE version (no minimum-bit change requirement): every field uses its
 * own natural C type (bool, enum, uint16_t/uint32_t, char). No bit-fields,
 * no masks/shifts, no packed bytes.
 */
#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_BARCODE_ALPHABET_SIZE   26u

#define LINE_BARCODE_PERIOD_MS       10u
#define LINE_BARCODE_LOST_MS         100u
#define LINE_BARCODE_SEARCH_MS       2000u
#define LINE_BARCODE_RUN_MAX_MS      1000u
#define LINE_BARCODE_QUIET_MS        100u
#define LINE_BARCODE_FRAME_RUNS      29u
/* SIDE cross-junction: scanner black for 4.5 narrow widths of the last
 * decoded barcode (18 mm crossing = 6 units, wide bar = 3). Before any
 * barcode is seen, a fixed time is used: 270 ms suits about 50 mm/s.
 */
#define LINE_BARCODE_JUNCTION_MS     270u
#define LINE_BARCODE_JUNCTION_NUM    9u
#define LINE_BARCODE_JUNCTION_DEN    2u

typedef enum
{
    LINE_BARCODE_LAYOUT_TRACK = 0,
    LINE_BARCODE_LAYOUT_SIDE
} line_barcode_layout_t;

typedef enum
{
    LINE_BARCODE_POS_FAR_LEFT = 0,
    LINE_BARCODE_POS_LEFT,
    LINE_BARCODE_POS_CENTRE,
    LINE_BARCODE_POS_RIGHT,
    LINE_BARCODE_POS_FAR_RIGHT,
    LINE_BARCODE_POS_JUNCTION,
    LINE_BARCODE_POS_NONE
} line_barcode_position_t;

typedef enum
{
    LINE_BARCODE_FOLLOW = 0,
    LINE_BARCODE_SEARCH,
    LINE_BARCODE_JUNCTION,
    LINE_BARCODE_SCANNING,
    LINE_BARCODE_PAUSED,
    LINE_BARCODE_FAULT
} line_barcode_state_t;

typedef enum
{
    LINE_BARCODE_CMD_LEFT = 0,
    LINE_BARCODE_CMD_RIGHT,
    LINE_BARCODE_CMD_STRAIGHT,
    LINE_BARCODE_CMD_UTURN,
    LINE_BARCODE_CMD_NONE
} line_barcode_cmd_t;

typedef enum
{
    LINE_BARCODE_EVT_STATUS = 0,
    LINE_BARCODE_EVT_JUNCTION,
    LINE_BARCODE_EVT_COMMAND,
    LINE_BARCODE_EVT_INVALID,
    LINE_BARCODE_EVT_LOST,
    LINE_BARCODE_EVT_FOUND,
    LINE_BARCODE_EVT_FAULT,
    LINE_BARCODE_EVT_BARCODE
} line_barcode_event_t;

typedef enum
{
    LINE_BARCODE_ERR_NONE = 0,
    LINE_BARCODE_ERR_LINE,
    LINE_BARCODE_ERR_QUEUE,
    LINE_BARCODE_ERR_TIMING
} line_barcode_fault_t;

/* One flag per IR sensor; true = black (after calibration). */
typedef struct
{
    bool b_left;
    bool b_centre;
    bool b_right;
} line_barcode_sensors_t;

/* Message sent through line_nav_q and returned by snapshots.
 * letter is 'A'..'Z' on BARCODE/COMMAND events, otherwise '\0'.
 * command is LINE_BARCODE_CMD_NONE except on COMMAND events.
 */
typedef struct
{
    line_barcode_sensors_t  sensors;
    line_barcode_position_t position;
    line_barcode_state_t    state;
    line_barcode_event_t    event;
    line_barcode_cmd_t      command;
    line_barcode_fault_t    fault;
    char                    letter;
} line_barcode_msg_t;

/* Letter-to-movement map: one entry per letter A..Z. */
typedef struct
{
    bool               b_valid;
    line_barcode_cmd_t command;
} line_barcode_map_entry_t;

typedef struct
{
    line_barcode_map_entry_t entry[LINE_BARCODE_ALPHABET_SIZE];
} line_barcode_map_t;

bool line_barcode_map_set(line_barcode_map_t *p_map, char letter,
                           line_barcode_cmd_t command);
void line_barcode_map_clear(line_barcode_map_t *p_map);

/* Letter remembered from the last barcode until the next junction. */
typedef struct
{
    bool b_pending;
    char letter;
} line_barcode_nav_t;

void line_barcode_nav_clear(line_barcode_nav_t *p_nav);
/* BARCODE (while following): store the letter, message unchanged.
 * JUNCTION with a stored letter: consume it; if mapped, the message becomes
 * a COMMAND carrying the letter and movement. Returns true on COMMAND.
 */
bool line_barcode_nav_update(line_barcode_nav_t *p_nav,
                             line_barcode_map_t const *p_map,
                             line_barcode_msg_t *p_msg);

/* Decoder working state, private to the owning task. */
typedef struct
{
    uint16_t                runs[LINE_BARCODE_FRAME_RUNS]; /* widths, ms */
    uint32_t                run_ms;
    uint32_t                lost_ms;
    uint32_t                search_ms;
    uint32_t                quiet_ms;
    line_barcode_sensors_t  candidate;
    line_barcode_sensors_t  stable;
    line_barcode_layout_t   layout;
    uint8_t                 run_count;
    bool                    b_black;
    bool                    b_capture;
    bool                    b_junction;
    bool                    b_confirmed;
    bool                    b_crossed;
    line_barcode_state_t    state;
    line_barcode_position_t last_position;
    uint16_t                last_narrow; /* narrow width of last barcode */
} line_barcode_ctx_t;

/* Call only from the owning task, or with a task-private test context. */
void line_barcode_reset(line_barcode_ctx_t *p_ctx);
/* SIDE (default): left scans barcode; centre/right track opposite line edges.
 * A cross-junction is all three black with the scanner held black longer
 * than any bar (see LINE_BARCODE_JUNCTION_MS).
 * TRACK: three-sensor tracking/junction candidates; barcode capture disabled.
 * Changing layout clears any partial frame and recovery timers.
 */
bool line_barcode_set_layout(line_barcode_ctx_t *p_ctx,
                             line_barcode_layout_t layout);
line_barcode_msg_t line_barcode_step(line_barcode_ctx_t *p_ctx,
                                    line_barcode_sensors_t sensors,
                                    uint16_t elapsed_ms);
/* widths alternate black/white, including two inter-character gaps.
 * Accepts *A* through *Z*, forward or reverse, no checksum.
 * Output is the letter 'A'..'Z'; no motion command is assigned here.
 */
bool line_barcode_decode(uint16_t const *p_runs, uint8_t count,
                         char *p_letter);
line_barcode_position_t line_barcode_position(line_barcode_sensors_t sensors);
line_barcode_msg_t line_barcode_make_msg(line_barcode_sensors_t sensors,
                                        line_barcode_position_t position,
                                        line_barcode_state_t state,
                                        line_barcode_event_t event,
                                        line_barcode_fault_t fault);

#if APP_TEST
void line_barcode_test(void);
#endif

#endif /* LINE_BARCODE_H */

/*** end of file ***/
