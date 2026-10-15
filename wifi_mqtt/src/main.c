
#include <stdio.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "wifi_manager.h"
#include "mqtt_manager.h"

#define TELEMETRY_INTERVAL_MS 5000
#define HEARTBEAT_INTERVAL_MS 10000
#define RETRY_INTERVAL_MS 5000

int main(void) {
    stdio_init_all();
    sleep_ms(3000);

    printf("Buddy 1 Starting...\n");

    if (!wifi_init()) {
        printf("Wi-Fi initialization failed.\n");
        return 1;
    }

    if (!mqtt_init()) {
        printf("MQTT initialization failed.\n");
        wifi_deinit();
        return 1;
    }

    uint32_t last_telemetry = 0;
    uint32_t last_heartbeat = 0;
    uint32_t last_wifi_retry = 0;
    uint32_t last_mqtt_retry = 0;

    while (true) {
        uint32_t now =
            to_ms_since_boot(get_absolute_time());

        // Step 1: Check Wi-Fi
        if (!wifi_is_connected()) {
            if ((uint32_t)(now - last_wifi_retry)
                >= RETRY_INTERVAL_MS) {

                wifi_connect();
                last_wifi_retry = to_ms_since_boot(get_absolute_time());
            }

            sleep_ms(100);
            continue;
        }

        // Step 2: Check MQTT
        if (!mqtt_is_connected()) {
            if ((uint32_t)(now - last_mqtt_retry)
                >= RETRY_INTERVAL_MS) {

                mqtt_connect();
                last_mqtt_retry = now;
            }

            sleep_ms(100);
            continue;
        }

        // Step 3: Publish telemetry
        if ((uint32_t)(now - last_telemetry)
            >= TELEMETRY_INTERVAL_MS) {

            mqtt_publish_telemetry(
                85,
                0.5f,
                "moving"
            );

            last_telemetry = now;
        }

        // Step 4: Publish heartbeat
        if ((uint32_t)(now - last_heartbeat)
            >= HEARTBEAT_INTERVAL_MS) {

            mqtt_publish_heartbeat();
            last_heartbeat = now;
        }

        sleep_ms(100);
    }

    return 0;
}
