#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "ultrasonic.h"

// ~30 ms covers the HC-SR04's maximum range (~4 m)
#define ECHO_TIMEOUT_US 30000

void ultrasonic_init(void)
{
    gpio_init(ULTRASONIC_TRIG_PIN);
    gpio_set_dir(ULTRASONIC_TRIG_PIN, GPIO_OUT);
    gpio_put(ULTRASONIC_TRIG_PIN, 0);
    gpio_init(ULTRASONIC_ECHO_PIN);
    gpio_set_dir(ULTRASONIC_ECHO_PIN, GPIO_IN);
}

// Function to get the pulse duration in microseconds (0 on timeout)
uint32_t get_pulse_duration_microsecond(void)
{
    // 10 us trigger pulse
    gpio_put(ULTRASONIC_TRIG_PIN, 1);
    sleep_us(10);
    gpio_put(ULTRASONIC_TRIG_PIN, 0);

    // Wait for echo to go high
    uint64_t wait_start = time_us_64();
    while (gpio_get(ULTRASONIC_ECHO_PIN) == 0)
    {
        if (time_us_64() - wait_start > ECHO_TIMEOUT_US)
        {
            return 0;
        }
    }

    // Time how long echo stays high
    uint64_t start_time = time_us_64();
    while (gpio_get(ULTRASONIC_ECHO_PIN) == 1)
    {
        if (time_us_64() - start_time > ECHO_TIMEOUT_US)
        {
            return 0;
        }
    }
    return (uint32_t)(time_us_64() - start_time);
}

// Function to get the distance in centimeters
float get_distance_cm(void)
{
    uint32_t pulse_duration = get_pulse_duration_microsecond();
    if (pulse_duration == 0)
    {
        return ULTRASONIC_NO_ECHO;
    }
    // speed of sound 0.0343 cm/us, halved for the round trip
    return pulse_duration * 0.0343f / 2;
}
