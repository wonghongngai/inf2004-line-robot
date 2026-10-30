/** @file line_barcode.h
 * @brief Portable line processing and Code 39 A-Z decoding.
 */
#ifndef LINE_BARCODE_H
#define LINE_BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_BARCODE_ALPHABET_SIZE   26u
#define LINE_BARCODE_LAYOUT_TRACK    0u
#define LINE_BARCODE_LAYOUT_SIDE     1u

#define LINE_BARCODE_PERIOD_MS       10u
#define LINE_BARCODE_DEBOUNCE        2u
#define LINE_BARCODE_LOST_MS         100u
#if defined(APP_ALL_TEST) && APP_ALL_TEST
/* Combined hardware trial: no line-search deadline. */
#define LINE_BARCODE_SEARCH_MS       0u
#else
#define LINE_BARCODE_SEARCH_MS       2000u
#endif
#define LINE_BARCODE_RUN_MAX_MS      1000u
#define LINE_BARCODE_QUIET_MS        100u
/* SIDE cross-junction: scanner black for 4.5 narrow widths of the last
 * decoded barcode (18 mm crossing = 6 units, wide bar = 3). Before any
 * barcode is seen, a fixed time is used: 270 ms suits about 50 mm/s.
 */
#define LINE_BARCODE_JUNCTION_MS     270u
#define LINE_BARCODE_JUNCTION_NUM    9u
#define LINE_BARCODE_JUNCTION_DEN    2u
#define LINE_BARCODE_FRAME_RUNS      29u
#define LINE_BARCODE_SENSOR_MASK     7u
#define LINE_BARCODE_LEFT            4u
#define LINE_BARCODE_CENTRE          2u
#define LINE_BARCODE_RIGHT           1u

#define LINE_BARCODE_FOLLOW          0u
#define LINE_BARCODE_SEARCH          1u
#define LINE_BARCODE_JUNCTION        2u
#define LINE_BARCODE_SCANNING        3u
#define LINE_BARCODE_PAUSED          4u
#define LINE_BARCODE_FAULT           5u

#define LINE_BARCODE_CMD_LEFT        0u
#define LINE_BARCODE_CMD_RIGHT       1u
#define LINE_BARCODE_CMD_STRAIGHT    2u
#define LINE_BARCODE_CMD_UTURN       3u

#define LINE_BARCODE_EVT_STATUS      0u
#define LINE_BARCODE_EVT_JUNCTION    1u
#define LINE_BARCODE_EVT_COMMAND     2u
#define LINE_BARCODE_EVT_INVALID     3u
#define LINE_BARCODE_EVT_LOST        4u
#define LINE_BARCODE_EVT_FOUND       5u
#define LINE_BARCODE_EVT_FAULT       6u
#define LINE_BARCODE_EVT_BARCODE     7u

/* Fault bitmask: one bit per fault, several may be set together. */
#define LINE_BARCODE_ERR_NONE        0u
#define LINE_BARCODE_ERR_LINE        1u  /* line lost longer than search */
#define LINE_BARCODE_ERR_QUEUE       2u  /* line_nav_q overflowed */
#define LINE_BARCODE_ERR_TIMING      4u  /* sampling interval missed */
#define LINE_BARCODE_ERR_MASK        7u

#define LINE_BARCODE_LETTER_MASK     0x1fu
#define LINE_BARCODE_FAULT_SHIFT     5u

/* Explicit wire layout, independent of ABI/bit-field ordering:
 * byte 0: sensors[2:0], position[5:3], command[7:6].
 * byte 1: state[2:0], event[5:3], reserved[7:6]=0.
 * byte 2: decoded letter[4:0] (A=0..Z=25), fault bitmask[7:5].
 * Letter valid only on BARCODE or COMMAND events.
 * Position 0..4 = far left, left, centred, right, far right;
 * 5 = ambiguous/junction; 6 = absent. Command valid only on COMMAND.
 */
typedef struct
{
    uint8_t data[3];
} line_barcode_msg_t;

/* 26 entries of {valid:1, command:2}: 78 bits in ten bytes. */
typedef struct
{
    uint8_t data[10];
} line_barcode_map_t;

bool line_barcode_map_set(line_barcode_map_t *p_map, char letter,
                           uint8_t command);
void line_barcode_map_clear(line_barcode_map_t *p_map);

/* Letter remembered from the last barcode until the next junction.
 * pending bit 7 = waiting, bits 4..0 = letter A=0..Z=25.
 */
typedef struct
{
    uint8_t pending;
} line_barcode_nav_t;

void line_barcode_nav_clear(line_barcode_nav_t *p_nav);
/* BARCODE (while following): store the letter, message unchanged.
 * JUNCTION with a stored letter: consume it; if mapped, the message becomes
 * a COMMAND carrying the letter and movement. Returns true on COMMAND.
 */
bool line_barcode_nav_update(line_barcode_nav_t *p_nav,
                             line_barcode_map_t const *p_map,
                             line_barcode_msg_t *p_msg);

typedef struct
{
    uint8_t runs[37]; /* 29 widths, 10 bits per width, 6 spare bits. */
    uint32_t run_ms : 14;
    uint32_t lost_ms : 7;
    uint32_t search_ms : 11;
    uint32_t quiet_ms : 14;
    uint32_t candidate : 3;
    uint32_t stable : 3;
    uint32_t layout : 1;
    uint32_t run_count : 5;
    uint32_t flags : 5;
    uint32_t state : 3;
    uint32_t last_position : 3;
    uint32_t last_narrow : 10; /* narrow width of last decoded barcode */
} line_barcode_ctx_t;

/* Call only from the owning task, or with a task-private test context. */
void line_barcode_reset(line_barcode_ctx_t *p_ctx);
/* SIDE (default): left scans barcode; centre/right track opposite line edges.
 * A cross-junction is all three black with the scanner held black longer
 * than any bar (see LINE_BARCODE_JUNCTION_MS).
 * TRACK: three-sensor tracking/junction candidates; barcode capture disabled.
 * Changing layout clears any partial frame and recovery timers.
 */
bool line_barcode_set_layout(line_barcode_ctx_t *p_ctx, uint8_t layout);
line_barcode_msg_t line_barcode_step(line_barcode_ctx_t *p_ctx,
                                    uint8_t sensors, uint16_t elapsed_ms);
/* widths alternate black/white, including two inter-character gaps.
 * Accepts *A* through *Z*, forward or reverse, no checksum.
 * Output is letter index 0..25; this function does not assign a motion command.
 */
bool line_barcode_decode(uint16_t const *p_runs, uint8_t count,
                         uint8_t *p_letter);
uint8_t line_barcode_position(uint8_t sensors);
line_barcode_msg_t line_barcode_pack(uint8_t sensors, uint8_t position,
                                    uint8_t command, uint8_t state,
                                    uint8_t event, uint8_t fault);

#if APP_TEST
void line_barcode_test(void);
#endif

#endif /* LINE_BARCODE_H */
