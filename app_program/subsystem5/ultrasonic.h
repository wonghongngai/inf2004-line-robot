#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>

#define ULTRASONIC_TRIG_PIN 7
#define ULTRASONIC_ECHO_PIN 6

// Returned by get_distance_cm() when no echo is received in time
#define ULTRASONIC_NO_ECHO (-1.0f)

// Configure the TRIG (output, low) and ECHO (input) pins
void ultrasonic_init(void);

// Echo pulse width in microseconds, or 0 on timeout
uint32_t get_pulse_duration_microsecond(void);

// Distance in centimetres, or ULTRASONIC_NO_ECHO on timeout
float get_distance_cm(void);

#endif
