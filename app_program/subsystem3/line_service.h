/** @file line_service.h
 * @brief micro T-Kernel interface to the single IR-owning task.
 *
 * BEFORE version (no minimum-bit change requirement): plain fields.
 */
#ifndef LINE_SERVICE_H
#define LINE_SERVICE_H

#include "../app.h"
#include "line_barcode.h"

#define LINE_SERVICE_PIN_LEFT       7u
#define LINE_SERVICE_PIN_CENTRE     27u
#define LINE_SERVICE_PIN_RIGHT      0u
#define LINE_SERVICE_PRIORITY       16
/* Room for about 12 queued events (message + buffer header each). */
#define LINE_SERVICE_QUEUE_EVENTS   12u
#define LINE_SERVICE_QUEUE_BYTES    \
    (LINE_SERVICE_QUEUE_EVENTS * (sizeof(line_barcode_msg_t) + 4u))

/* Message buffer named line_nav_q in the architecture. Use receive API,
 * never write to it. One Vehicle Controller task consumes event messages.
 */
extern ID g_h_line_nav_q;

ER line_service_init(void);
ER line_service_start(void);
/* Task context only. Give the raw GPIO level of each sensor (true = pin
 * reads 1) with all sensors on black, then all on white. Each sensor must
 * read differently on black and white. Starts paused.
 */
ER line_service_calibrate(line_barcode_sensors_t black_raw,
                          line_barcode_sensors_t white_raw);
/* Enable following after calibration; false pauses sensing decisions during
 * turns/avoidance. Resume resets partial barcodes and line-loss timers.
 */
ER line_service_enable(bool b_enable);
/* Configure only while paused. SIDE is the sample's offset-barcode layout;
 * TRACK uses all three sensors for line/junction candidates and no barcode.
 */
ER line_service_set_layout(line_barcode_layout_t layout);
/* Letter-to-movement mapping, allowed at any time (e.g. from the MQTT task).
 * A decoded letter is stored; at the next junction a mapped letter gives a
 * COMMAND event, an unmapped one a plain JUNCTION event.
 * set_mapping replaces the whole map with four distinct letters 'A'..'Z';
 * on E_PAR the old map is kept.
 */
ER line_service_map_letter(char letter, line_barcode_cmd_t command);
ER line_service_set_mapping(char left, char right, char straight, char uturn);
ER line_service_clear_map(void);
/* Returns E_TMOUT if the last sampling heartbeat is over 30 ms old.
 * Always check the return code before using a snapshot.
 */
ER line_service_snapshot(line_barcode_msg_t *p_msg);
ER line_service_receive(line_barcode_msg_t *p_msg, TMO timeout);

#endif /* LINE_SERVICE_H */

/*** end of file ***/
