
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"

#include "mqtt_manager.h"
#include "wifi_manager.h"

// MQTT broker configuration
#define MQTT_BROKER_IP "192.168.1.10"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID "buddy1_robot"

// MQTT topics
#define MQTT_TELEMETRY_TOPIC "robot/telemetry"
#define MQTT_HEARTBEAT_TOPIC "robot/heartbeat"

static mqtt_client_t *mqtt_client = NULL;
static bool mqtt_connecting = false;
#define MQTT_CONNECT_TIMEOUT_MS 30000U
static uint32_t connect_started_ms = 0;

// Called when MQTT connection status changes
static void mqtt_connection_callback(
    mqtt_client_t *client,
    void *arg,
    mqtt_connection_status_t status
) {
    (void)client;
    (void)arg;

    mqtt_connecting = false;

    if (status == MQTT_CONNECT_ACCEPTED) {
        printf("MQTT connected!\n");
    } else {
        printf("MQTT disconnected/status: %d\n",
               (int)status);
    }
}

// Called after a publish request completes
static void mqtt_publish_callback(
    void *arg,
    err_t result
) {
    (void)arg;

    if (result == ERR_OK) {
        printf("MQTT publish acknowledged.\n");
    } else {
        printf("MQTT publish failed: %d\n",
               (int)result);
    }
}

bool mqtt_init(void) {
    if (mqtt_client != NULL) {
        return true;
    }

    cyw43_arch_lwip_begin();
    mqtt_client = mqtt_client_new();
    cyw43_arch_lwip_end();

    if (mqtt_client == NULL) {
        printf("MQTT initialization failed!\n");
        return false;
    }

    printf("MQTT initialized!\n");
    return true;
}

bool mqtt_is_connected(void) {
    if (mqtt_client == NULL) {
        return false;
    }

    cyw43_arch_lwip_begin();
    bool connected =
        mqtt_client_is_connected(mqtt_client);
    cyw43_arch_lwip_end();

    return connected;
}

bool mqtt_connect(void) {
    if (!wifi_is_connected() ||
        mqtt_client == NULL) {
        return false;
    }

    uint32_t now = to_ms_since_boot(get_absolute_time());

    ip_addr_t broker_ip;

    if (!ipaddr_aton(MQTT_BROKER_IP, &broker_ip)) {
        printf("Invalid MQTT broker IP!\n");
        return false;
    }

    struct mqtt_connect_client_info_t client_info = {
        .client_id = MQTT_CLIENT_ID,
        .client_user = NULL,
        .client_pass = NULL,
        .keep_alive = 60,
        .will_topic = NULL,
        .will_msg = NULL,
        .will_qos = 0,
        .will_retain = 0
    };

    printf("Connecting to MQTT broker...\n");

    cyw43_arch_lwip_begin();

    if (mqtt_client_is_connected(mqtt_client)) {
        cyw43_arch_lwip_end();
        return true;
    }
    if (mqtt_connecting) {
        if ((uint32_t)(now - connect_started_ms) < MQTT_CONNECT_TIMEOUT_MS) {
            cyw43_arch_lwip_end();
            return false;
        }
        mqtt_disconnect(mqtt_client);
        mqtt_connecting = false;
    }

    mqtt_connecting = true;
    connect_started_ms = now;
    err_t result = mqtt_client_connect(
        mqtt_client,
        &broker_ip,
        MQTT_BROKER_PORT,
        mqtt_connection_callback,
        NULL,
        &client_info
    );

    if (result != ERR_OK) {
        mqtt_connecting = false;
    }
    cyw43_arch_lwip_end();

    if (result != ERR_OK) {
        printf("MQTT connect error: %d\n",
               (int)result);
        return false;
    }

    // Connection completes asynchronously.
    return true;
}

bool mqtt_publish_message(
    const char *topic,
    const char *message
) {
    if (mqtt_client == NULL || topic == NULL || message == NULL ||
        topic[0] == '\0' || strlen(message) > UINT16_MAX) {
        return false;
    }

    cyw43_arch_lwip_begin();
    if (!mqtt_client_is_connected(mqtt_client)) {
        cyw43_arch_lwip_end();
        return false;
    }

    err_t result = mqtt_publish(
        mqtt_client,
        topic,
        message,
        (u16_t)strlen(message),
        1,
        0,
        mqtt_publish_callback,
        NULL
    );

    cyw43_arch_lwip_end();

    if (result != ERR_OK) {
        printf("Publish request failed: %d\n",
               (int)result);
        return false;
    }

    return true;
}

void mqtt_publish_telemetry(
    int battery,
    float speed,
    const char *status
) {
    if (status == NULL || battery < 0 || battery > 100 || !isfinite(speed)) {
        return;
    }
    // Reject strings requiring JSON escaping rather than emit malformed JSON.
    for (const unsigned char *p = (const unsigned char *)status; *p; ++p) {
        if (*p < 0x20 || *p == '"' || *p == '\\') {
            return;
        }
    }
    char payload[160];

    int length = snprintf(
        payload,
        sizeof(payload),
        "{\"battery\":%d,\"speed\":%.2f,"
        "\"status\":\"%s\"}",
        battery,
        speed,
        status
    );

    if (length < 0 || (size_t)length >= sizeof(payload)) {
        printf("Telemetry payload too long.\n");
        return;
    }

    mqtt_publish_message(
        MQTT_TELEMETRY_TOPIC,
        payload
    );
}

void mqtt_publish_heartbeat(void) {
    mqtt_publish_message(
        MQTT_HEARTBEAT_TOPIC,
        "{\"status\":\"online\"}"
    );
}
