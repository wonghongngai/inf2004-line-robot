
#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <stdbool.h>

bool mqtt_init(void);
bool mqtt_connect(void);
bool mqtt_is_connected(void);

bool mqtt_publish_message(
    const char *topic,
    const char *message
);

void mqtt_publish_telemetry(
    int battery,
    float speed,
    const char *status
);

void mqtt_publish_heartbeat(void);

#endif
