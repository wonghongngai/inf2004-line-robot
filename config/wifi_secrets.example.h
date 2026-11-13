/*
 * wifi_secrets.example.h - template for config/wifi_secrets.h
 *
 * Copy this file to config/wifi_secrets.h and fill in your own values.
 * config/wifi_secrets.h is gitignored: NEVER commit real credentials.
 * Needed only for builds with WIFI=cyw43 (and NET=lwip MQTT=1 for MQTT).
 */
#ifndef WIFI_SECRETS_H
#define WIFI_SECRETS_H

/* ---- Wi-Fi (lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c) ---- */
#define WIFI_SSID        "your-network-name"
#define WIFI_PASSWORD    "your-network-password"
/* Optional. Default: CYW43_AUTH_WPA2_MIXED_PSK.
 * #define WIFI_AUTH_MODE CYW43_AUTH_WPA2_AES_PSK
 * Optional. 1 = scan and print nearby networks when joining fails.
 * #define WIFI_DIAGNOSTIC_SCAN 1
 */

/* ---- MQTT (app_program/subsystem1/mqtt_manager.c) ---- */
#define MQTT_BROKER_IP   "192.168.1.100"   /* the broker's IPv4 address, not a host name */
#define MQTT_BROKER_PORT 1883              /* tools/mosquitto-local.conf listens on 1883 */
#define MQTT_CLIENT_ID   "group17-robot"   /* also the topic prefix: <id>/telemetry, <id>/command, <id>/mapping */
#define MQTT_TOPIC       MQTT_CLIENT_ID "/status"  /* heartbeat and last-will topic */
/* Optional topic overrides (defaults shown):
 * #define MQTT_TELEMETRY_TOPIC MQTT_CLIENT_ID "/telemetry"
 * #define MQTT_COMMAND_TOPIC   MQTT_CLIENT_ID "/command"
 * #define MQTT_MAPPING_TOPIC   MQTT_CLIENT_ID "/mapping"
 */

#endif /* WIFI_SECRETS_H */
