#if TM_MQTT
/*
 *  mqtt_manager.c
 *  MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *  Runs entirely on the single processor-1-owned task that already owns
 *  the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *  cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() each poll
 *  while the link is up. mqtt.c's own keepalive/ping/retry
 *  timers, and the heartbeat below, both ride the same sys_check_timeouts()
 *  polling that task already does for DHCP and TCP -- nothing extra runs
 *  on any other task or core.
 *
 *  This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *  (unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *  file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 */

#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include "communication_internal.h"
#include <tm/tmonitor.h>

#define MQTT_UTK_HEARTBEAT_MS   5000
#define MQTT_UTK_KEEPALIVE_S    60
#define MQTT_UTK_RETRY_MS       3000

#ifndef MQTT_COMMAND_TOPIC
#define MQTT_COMMAND_TOPIC MQTT_CLIENT_ID "/command"
#endif
#ifndef MQTT_TELEMETRY_TOPIC
#define MQTT_TELEMETRY_TOPIC MQTT_CLIENT_ID "/telemetry"
#endif

static mqtt_client_t    *client;
static ip_addr_t    broker_addr;
static u8_t     connecting;

/* Two command bits are stored separately from receive protocol state.
 * Owned solely by the Wi-Fi task. Zero is a valid command, not a terminator. */
static unsigned char rx_command;
#define RX_VALID 0x04U
#define RX_HAVE_DATA 0x08U
static unsigned char rx_flags;

static void heartbeat_timeout(void *arg);

static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    unsigned int i = 0;
    (void)arg;
    while (topic[i] != '\0' && MQTT_COMMAND_TOPIC[i] != '\0' &&
           topic[i] == MQTT_COMMAND_TOPIC[i]) {
        ++i;
    }
    rx_flags = (tot_len == S1_COMMAND_BYTES &&
                topic[i] == '\0' && MQTT_COMMAND_TOPIC[i] == '\0') ? RX_VALID : 0U;
    rx_command = 0;
}

static void incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    (void)arg;
    if ((rx_flags & RX_VALID) != 0U && len != 0U) {
        if (len != S1_COMMAND_BYTES || (rx_flags & RX_HAVE_DATA) != 0U ||
            (data[0] & ~S1_COMMAND_MASK) != 0U) {
            rx_flags = 0;
        } else {
            rx_command = data[0];
            rx_flags |= RX_HAVE_DATA;
        }
    }
    if ((flags & MQTT_DATA_FLAG_LAST) != 0U) {
        if ((rx_flags & (RX_VALID | RX_HAVE_DATA)) == (RX_VALID | RX_HAVE_DATA)) {
            s1_deliver_command(&rx_command, S1_COMMAND_BYTES);
        } else {
            tm_printf((UB *)"mqtt: invalid binary command rejected\n");
        }
        rx_flags = 0;
    }
}

static void subscribe_done_cb(void *arg, err_t result)
{
    (void)arg;
    if(result == ERR_OK) {
        tm_printf((UB*)"mqtt: subscribed to " MQTT_COMMAND_TOPIC "\n");
    } else {
        tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
    }
}

static void publish_done_cb(void *arg, err_t result)
{
    (void)arg;
    if(result != ERR_OK) {
        tm_printf((UB*)"mqtt: publish failed, err=%d\n", (int)result);
    }
}

static void heartbeat_timeout(void *arg)
{
    const unsigned char payload = S1_HEARTBEAT_ONLINE;
    (void)arg;
    if (mqtt_client_is_connected(client)) {
        (void)mqtt_publish(client, MQTT_TOPIC, &payload, S1_HEARTBEAT_BYTES,
                          1, 1, publish_done_cb, NULL);
    }

    /* Re-arms itself every call -- mqtt.c's connection callback (below)
     * only starts this chain once, on the transition into MQTT_CONNECT_
     * ACCEPTED, so there is exactly one heartbeat chain alive at a time. */
    sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

static void retry_timeout(void *arg)
{
    (void)arg;
    connecting = 0;
    mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg, mqtt_connection_status_t status)
{
    (void)c;
    (void)arg;

    if(status == MQTT_CONNECT_ACCEPTED) {
        err_t   err;

        /* No longer *connecting* -- clear the guard so that if this
         * connection later drops, the poll loop's mqtt_utk_start() call
         * is allowed to reconnect. Leaving it set would wedge the client
         * permanently after the first disconnect. */
        connecting = 0;
        sys_untimeout(retry_timeout, NULL);

        tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

        /* Reinstall command subscription after every reconnect. */
        mqtt_set_inpub_callback(client, incoming_publish_cb,
                    incoming_data_cb, NULL);

        err = mqtt_subscribe(client, MQTT_COMMAND_TOPIC, 0 /* qos */,
                     subscribe_done_cb, NULL);
        if(err != ERR_OK) {
            tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
                  (int)err);
        }

        /* Drop any chain left over from a previous connection before
         * arming a new one. Each reconnect would otherwise add another
         * self-perpetuating heartbeat, and every one of them holds a
         * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
        sys_untimeout(heartbeat_timeout, NULL);
        sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
    } else {
        /* mqtt_client_connect() reconnects on its own for a dropped
         * keepalive; a rejected/never-established connection does not
         * retry itself, so do that here -- on a delay, so a broker
         * that keeps rejecting (e.g. bad client ID) can't turn this
         * into a tight reconnect loop. */
        tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
              (int)status, MQTT_UTK_RETRY_MS / 1000);
        connecting = 1; /* Guard poll-loop retries until the timer fires. */
        sys_untimeout(heartbeat_timeout, NULL);
        sys_untimeout(retry_timeout, NULL);
        sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
    }
}

static void publish_queued_telemetry(void)
{
    static unsigned char pending[S1_TELEMETRY_BYTES];
    static int length;
    err_t result;
    if (length <= 0) {
        length = s1_take_telemetry(pending);
    }
    if (length > 0) {
        result = mqtt_publish(client, MQTT_TELEMETRY_TOPIC, pending,
                              (u16_t)length, 1, 0, publish_done_cb, NULL);
        if (result == ERR_OK) {
            length = 0;
        }
    }
}

void mqtt_utk_start(void)
{
    struct mqtt_connect_client_info_t   client_info = {0};
    err_t                   err;

    if (client != NULL && mqtt_client_is_connected(client)) {
        publish_queued_telemetry();
        return;
    }
    if(connecting) {
        return;
    }

    if(client == NULL) {
        client = mqtt_client_new();
        if(client == NULL) {
            tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
            return;
        }
    }

    if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
        tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
        return;
    }
    IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

    client_info.client_id = MQTT_CLIENT_ID;
    client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;
    client_info.will_topic = MQTT_TOPIC;
    client_info.will_msg = "\x00";
    client_info.will_msg_len = S1_HEARTBEAT_BYTES;
    client_info.will_qos = 1;
    client_info.will_retain = 1;

    connecting = 1;
    err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
                  connection_cb, NULL, &client_info);
    if(err != ERR_OK) {
        tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
        connecting = 1;
        sys_untimeout(retry_timeout, NULL);
        sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
    }
}

#endif /* TM_MQTT */
