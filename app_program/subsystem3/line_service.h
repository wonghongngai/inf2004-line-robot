/** @file line_service.h
 * @brief micro T-Kernel interface to the single IR-owning task.
 */
#ifndef LINE_SERVICE_H
#define LINE_SERVICE_H

#include "../app.h"
#include "line_barcode.h"

#define LINE_SERVICE_PIN_LEFT       28u
#define LINE_SERVICE_PIN_CENTRE     27u
#define LINE_SERVICE_PIN_RIGHT      0u
#define LINE_SERVICE_PRIORITY       16
#define LINE_SERVICE_QUEUE_BYTES    96

/* Message buffer named line_nav_q in the architecture. Use receive API,
 * never write to it. One Vehicle Controller task consumes event messages.
 */
extern ID g_h_line_nav_q;

ER line_service_init(void);
ER line_service_start(void);
/* Task context only. Supply raw GPIO levels measured with all sensors on
 * black, then all on white (left bit2, centre bit1, right bit0).
 * Each sensor must have opposite black/white levels. Starts paused.
 */
ER line_service_calibrate(uint8_t black_raw, uint8_t white_raw);
/* Enable following after calibration; false pauses sensing decisions during
 * turns/avoidance. Resume resets partial barcodes and line-loss timers.
 */
ER line_service_enable(bool b_enable);
/* Configure only while paused. SIDE is the sample's offset-barcode layout;
 * TRACK uses all three sensors for line/junction candidates and no barcode.
 */
ER line_service_set_layout(uint8_t layout);
/* Letter-to-movement mapping, allowed at any time (e.g. from the MQTT task).
 * A decoded letter is stored; at the next junction a mapped letter gives a
 * COMMAND event, an unmapped one a plain JUNCTION event.
 * set_mapping replaces the whole map with four distinct letters 'A'..'Z';
 * on E_PAR the old map is kept.
 */
ER line_service_map_letter(char letter, uint8_t command);
ER line_service_set_mapping(char left, char right, char straight, char uturn);
ER line_service_clear_map(void);
/* Current map entry for letter 0..25 (A=0): E_OK with *p_command set,
 * E_NOEXS if unmapped, E_PAR on bad arguments. Any task.
 */
ER line_service_lookup(uint8_t letter, uint8_t *p_command);
/* Returns E_TMOUT if the last sampling heartbeat is over 30 ms old.
 * Always check the return code before using a snapshot.
 */
ER line_service_snapshot(line_barcode_msg_t *p_msg);
/* Sampling interval recorded at the first decoder fault; no console I/O. */
uint32_t line_service_fault_interval_ms(void);
/* Diagnostic counts of accepted bars/white runs; letter 0..25, 255 if none.
 * Caller supplies three non-null output pointers. */
/* Pop a completed debounced run. Non-null outputs; dropped is cumulative.
 * White runs may be margins; this is a diagnostic stream, not a frame. */
bool line_service_barcode_run(uint32_t *ms, unsigned *black, uint32_t *dropped);
/* Current accepted level and time since its last accepted change.
 * Outputs are meaningful while the service is enabled. Non-null pointers. */
/* Sampled digital input and session HIGH/LOW edge totals, before debounce. */
void line_service_barcode_raw(unsigned *level, unsigned *rises, unsigned *falls);
void line_service_barcode_level(unsigned *black, uint32_t *age_ms);
void line_service_barcode_debug(unsigned *black, unsigned *white, uint8_t *letter);
ER line_service_receive(line_barcode_msg_t *p_msg, TMO timeout);
/* Latest completed/rejected capture, copied outside the sampling task.
 * sequence lets the logger detect a newer frame. Widths are elapsed ms. */
typedef struct {
    uint32_t sequence;
    uint16_t widths[LINE_BARCODE_FRAME_RUNS];
    uint8_t count, letter, event;
} line_service_barcode_frame_t;
void line_service_barcode_frame(line_service_barcode_frame_t *frame);

#endif /* LINE_SERVICE_H */
