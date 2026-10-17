#include <stdio.h>
#include "pico/stdlib.h"
#include "ultrasonic.h"

// Test program: print the distance in centimeters once per second
int main()
{
    stdio_init_all();
    ultrasonic_init();

    while (1)
    {
        float distance = get_distance_cm();
        if (distance < 0)
        {
            printf("No echo\n");
        }
        else
        {
            printf("Distance: %.1f cm\n", distance);
        }
        sleep_ms(1000);
    }
}
