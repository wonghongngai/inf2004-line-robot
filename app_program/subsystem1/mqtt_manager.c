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
#include "lwip/sys.h" /* sys_now() for connect timestamps */
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include "communication_internal.h"
#include "../app_log.h" /* LOG_xxx console macros; not app.h (size_t clash) */

#define MQTT_UTK_HEARTBEAT_MS 2000
#define MQTT_UTK_KEEPALIVE_S  60
#define MQTT_UTK_RETRY_MS     3000

#ifndef MQTT_COMMAND_TOPIC
#define MQTT_COMMAND_TOPIC MQTT_CLIENT_ID "/command"
#endif
#ifndef MQTT_MAPPING_TOPIC
#define MQTT_MAPPING_TOPIC MQTT_CLIENT_ID "/mapping"
#endif
#ifndef MQTT_TELEMETRY_TOPIC
#define MQTT_TELEMETRY_TOPIC MQTT_CLIENT_ID "/telemetry"
#endif
#ifndef MQTT_BARCODE_TOPIC
#define MQTT_BARCODE_TOPIC MQTT_CLIENT_ID "/barcode"
#endif

#define BARCODE_NONE    0U /* nothing pending */
#define BARCODE_WAITING 1U /* pending, not yet accepted by lwIP */
#define BARCODE_SENT    2U /* published, waiting for the broker's PUBACK */
/* One event taken from the queue, kept until the broker acknowledges it.
 * Owned solely by the Wi-Fi task. */
static S1_BARCODE_RESULT g_barcode_pending;
static unsigned char     g_barcode_state;

static mqtt_client_t * g_p_client;
static ip_addr_t       g_broker_addr;
static u8_t            g_connecting;

/* One incoming binary payload, stored separately from receive protocol state.
 * Owned solely by the Wi-Fi task. Zero is valid data, not a terminator. */
#define RX_NONE        0U
#define RX_COMMAND     1U
#define RX_MAPPING     2U
#define RX_BAD_COMMAND 3U /* command topic, wrong length: counted, not queued */
static unsigned char g_rx_payload[S1_MAPPING_BYTES];
static unsigned char g_rx_kind;
static unsigned char g_rx_length;
static unsigned char g_rx_log; /* this message may print its own lines */

static void heartbeat_timeout(void * p_arg);

static int topic_is(const char * p_topic, const char * p_name)
{
    unsigned int idx = 0;
    while (p_topic[idx] != '\0' && p_name[idx] != '\0' &&
           p_topic[idx] == p_name[idx])
    {
        ++idx;
    }
    return p_topic[idx] == '\0' && p_name[idx] == '\0';
}

static void incoming_publish_cb(void * p_arg, const char * p_topic,
                                u32_t tot_len)
{
    (void)p_arg;
    g_rx_length = 0;
    g_rx_kind   = RX_NONE;
    g_rx_log    = 1;
    if (topic_is(p_topic, MQTT_COMMAND_TOPIC))
    {
        /* Every command message is counted (S1-U17); only a few per
         * window print, so a flood cannot tie the task up in printing. */
        g_rx_kind = tot_len == S1_COMMAND_BYTES ? RX_COMMAND : RX_BAD_COMMAND;
        g_rx_log  = (unsigned char)s1_command_log_ok();
    }
    else if (topic_is(p_topic, MQTT_MAPPING_TOPIC) &&
             tot_len == S1_MAPPING_BYTES)
    {
        g_rx_kind = RX_MAPPING;
    }
    else
    {
        /* No action needed in any other case. */
    }
    if (g_rx_log)
    {
        LOG_INFO(TAG_S1_MQTT, "RX topic=%s length=%lu", p_topic,
                 (unsigned long)tot_len);
    }
}

static void incoming_data_cb(void * p_arg, const u8_t * p_data, u16_t len,
                             u8_t flags)
{
    unsigned int expected =
        g_rx_kind == RX_MAPPING ? S1_MAPPING_BYTES : S1_COMMAND_BYTES;
    unsigned int idx;
    (void)p_arg;
    if (g_rx_kind == RX_COMMAND || g_rx_kind == RX_MAPPING)
    {
        if (len > expected - g_rx_length)
        {
            g_rx_kind = g_rx_kind == RX_COMMAND ? RX_BAD_COMMAND : RX_NONE;
        }
        else
        {
            for (idx = 0; idx < len; ++idx)
            {
                g_rx_payload[g_rx_length + idx] = p_data[idx];
            }
            g_rx_length = (unsigned char)(g_rx_length + len);
        }
    }
    if ((flags & MQTT_DATA_FLAG_LAST) != 0U)
    {
        S1_COMMAND command = {{g_rx_payload[0]}};
        if (g_rx_kind == RX_COMMAND && g_rx_length == S1_COMMAND_BYTES &&
            s1_command_valid(&command))
        {
            if (g_rx_log)
            {
                LOG_INFO(TAG_S1_MQTT, "RX command=%u",
                         (unsigned int)g_rx_payload[0]);
            }
            s1_deliver_command(g_rx_payload, S1_COMMAND_BYTES);
        }
        else if (g_rx_kind == RX_COMMAND || g_rx_kind == RX_BAD_COMMAND)
        {
            /* Short, long or reserved bits set: s1_deliver_command counts it. */
            if (g_rx_log)
            {
                LOG_WARN(TAG_S1_MQTT, "invalid binary command rejected");
            }
            s1_deliver_command(g_rx_payload,
                               g_rx_kind == RX_COMMAND ? g_rx_length : 0U);
        }
        else if (g_rx_kind == RX_MAPPING && g_rx_length == S1_MAPPING_BYTES)
        {
            s1_deliver_mapping(g_rx_payload, S1_MAPPING_BYTES); /* validates */
        }
        else
        {
            LOG_WARN(TAG_S1_MQTT, "invalid binary command/mapping rejected");
        }
        g_rx_kind   = RX_NONE;
        g_rx_length = 0;
    }
}

static void subscribe_done_cb(void * p_arg, err_t result)
{
    if (result == ERR_OK)
    {
        LOG_INFO(TAG_S1_MQTT, "subscribed to %s", (const char *)p_arg);
    }
    else
    {
        LOG_ERR(TAG_S1_MQTT, "subscribe %s failed, err=%d", (const char *)p_arg,
                (int)result);
    }
}

static void publish_done_cb(void * p_arg, err_t result)
{
    (void)p_arg;
    if (result != ERR_OK)
    {
        LOG_ERR(TAG_S1_MQTT, "publish failed, err=%d", (int)result);
    }
}

static void barcode_done_cb(void * p_arg, err_t result)
{
    unsigned int frame = (unsigned int)(mem_ptr_t)p_arg;
    /* Ignore a late callback for an event that was already re-armed. */
    if (g_barcode_state != BARCODE_SENT || frame != g_barcode_pending.id)
    {
        return;
    }
    if (result == ERR_OK)
    {
        LOG_INFO(TAG_S1_MQTT, "barcode acked frame=%u", frame);
        g_barcode_state = BARCODE_NONE;
    }
    else
    {
        LOG_WARN(TAG_S1_MQTT,
                 "barcode publish frame=%u failed, err=%d; retrying", frame,
                 (int)result);
        g_barcode_state = BARCODE_WAITING;
    }
}

static void publish_queued_barcode(void)
{
    S1_BARCODE_PACKET packet;
    err_t             result;
    if (g_barcode_state == BARCODE_NONE)
    {
        if (s1_take_barcode(&g_barcode_pending) <= 0)
        {
            return;
        }
        g_barcode_state = BARCODE_WAITING;
    }
    if (g_barcode_state != BARCODE_WAITING)
    {
        return;
    }
    if (!s1_barcode_pack(&g_barcode_pending, &packet))
    {
        g_barcode_state = BARCODE_NONE; /* S1 validated it; never expected */
        return;
    }
    /* mqtt_publish copies the payload, so the stack buffer is safe. */
    result = mqtt_publish(g_p_client, MQTT_BARCODE_TOPIC, packet.bytes,
                          (u16_t)S1_BARCODE_BYTES, 1, 0, barcode_done_cb,
                          (void *)(mem_ptr_t)g_barcode_pending.id);
    if (result == ERR_OK)
    {
        g_barcode_state = BARCODE_SENT;
    }
}

static void heartbeat_timeout(void * p_arg)
{
    const unsigned char payload = S1_HEARTBEAT_ONLINE;
    (void)p_arg;
    if (mqtt_client_is_connected(g_p_client))
    {
        (void)mqtt_publish(g_p_client, MQTT_TOPIC, &payload, S1_HEARTBEAT_BYTES,
                           1, 1, publish_done_cb, NULL);
    }
    s1_command_stats_report();

    /* Re-arms itself every call -- mqtt.c's connection callback (below)
     * only starts this chain once, on the transition into MQTT_CONNECT_
     * ACCEPTED, so there is exactly one heartbeat chain alive at a time. */
    sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

static void retry_timeout(void * p_arg)
{
    (void)p_arg;
    /* The Wi-Fi owner starts the next attempt only when an IP is ready. */
    g_connecting = 0;
}

static void connection_cb(mqtt_client_t * p_client, void * p_arg,
                          mqtt_connection_status_t status)
{
    (void)p_client;
    (void)p_arg;

    if (status == MQTT_CONNECT_ACCEPTED)
    {
        err_t err;

        /* No longer *connecting* -- clear the guard so that if this
         * connection later drops, the poll loop's mqtt_utk_start() call
         * is allowed to reconnect. Leaving it set would wedge the client
         * permanently after the first disconnect. */
        g_connecting = 0;
        sys_untimeout(retry_timeout, NULL);

        /* t= is ms since boot (lwIP sys_now), for S1-U11 and S1-U13. */
        LOG_INFO(TAG_S1_MQTT, "connected to " MQTT_BROKER_IP " t=%u ms",
                 (unsigned int)sys_now());

        /* lwIP drops unacknowledged requests on disconnect without calling
         * their callbacks, so resend the pending barcode event. */
        if (g_barcode_state == BARCODE_SENT)
        {
            g_barcode_state = BARCODE_WAITING;
        }

        /* Reinstall command and mapping subscriptions after every reconnect. */
        mqtt_set_inpub_callback(g_p_client, incoming_publish_cb,
                                incoming_data_cb, NULL);

        err = mqtt_subscribe(g_p_client, MQTT_COMMAND_TOPIC, 1 /* qos */,
                             subscribe_done_cb, (void *)MQTT_COMMAND_TOPIC);
        if (err != ERR_OK)
        {
            LOG_ERR(TAG_S1_MQTT, "mqtt_subscribe() failed, err=%d", (int)err);
        }
        /* QoS 1: a lost map would leave every barcode unmapped. */
        err = mqtt_subscribe(g_p_client, MQTT_MAPPING_TOPIC, 1 /* qos */,
                             subscribe_done_cb, (void *)MQTT_MAPPING_TOPIC);
        if (err != ERR_OK)
        {
            LOG_ERR(TAG_S1_MQTT, "mapping mqtt_subscribe() failed, err=%d",
                    (int)err);
        }

        /* Drop any chain left over from a previous connection before
         * arming a new one. Each reconnect would otherwise add another
         * self-perpetuating heartbeat, and every one of them holds a
         * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
        sys_untimeout(heartbeat_timeout, NULL);
        sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
    }
    else
    {
        /* mqtt_client_connect() reconnects on its own for a dropped
         * keepalive; a rejected/never-established connection does not
         * retry itself, so do that here -- on a delay, so a broker
         * that keeps rejecting (e.g. bad client ID) can't turn this
         * into a tight reconnect loop. */
        LOG_WARN(TAG_S1_MQTT, "connection status %d t=%u ms, retrying in %ds",
                 (int)status, (unsigned int)sys_now(),
                 MQTT_UTK_RETRY_MS / 1000);
        g_connecting = 1; /* Guard poll-loop retries until the timer fires. */
        sys_untimeout(heartbeat_timeout, NULL);
        sys_untimeout(retry_timeout, NULL);
        sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
    }
}

static void publish_queued_telemetry(void)
{
    static unsigned char pending[S1_TELEMETRY_BYTES];
    static int           length;
    err_t                result;
    if (length <= 0)
    {
        length = s1_take_telemetry(pending);
    }
    if (length > 0)
    {
        result = mqtt_publish(g_p_client, MQTT_TELEMETRY_TOPIC, pending,
                              (u16_t)length, 1, 0, publish_done_cb, NULL);
        if (result == ERR_OK)
        {
            length = 0;
        }
    }
}

void mqtt_utk_network_down(void)
{
    sys_untimeout(retry_timeout, NULL);
    sys_untimeout(heartbeat_timeout, NULL);
    if (g_p_client != NULL &&
        (g_connecting || mqtt_client_is_connected(g_p_client)))
    {
        LOG_WARN(TAG_S1_MQTT, "Wi-Fi unavailable; waiting for IP");
        mqtt_disconnect(g_p_client);
    }
    g_connecting = 0;
}

void mqtt_utk_start(void)
{
    struct mqtt_connect_client_info_t client_info = {0};
    err_t                             err;

    if (g_p_client != NULL && mqtt_client_is_connected(g_p_client))
    {
        publish_queued_telemetry();
        publish_queued_barcode();
        return;
    }
    if (g_connecting)
    {
        return;
    }

    if (g_p_client == NULL)
    {
        g_p_client = mqtt_client_new();
        if (g_p_client == NULL)
        {
            LOG_ERR(TAG_S1_MQTT, "mqtt_client_new() failed");
            return;
        }
    }

    if (!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&g_broker_addr)))
    {
        LOG_ERR(TAG_S1_MQTT, "bad MQTT_BROKER_IP");
        return;
    }
    IP_SET_TYPE(&g_broker_addr, IPADDR_TYPE_V4);

    client_info.client_id    = MQTT_CLIENT_ID;
    client_info.keep_alive   = MQTT_UTK_KEEPALIVE_S;
    client_info.will_topic   = MQTT_TOPIC;
    client_info.will_msg     = "\x00";
    client_info.will_msg_len = S1_HEARTBEAT_BYTES;
    client_info.will_qos     = 1;
    client_info.will_retain  = 1;

    LOG_INFO(TAG_S1_MQTT,
             "connecting to " MQTT_BROKER_IP ":%u as " MQTT_CLIENT_ID "",
             (unsigned)MQTT_BROKER_PORT);
    g_connecting = 1;
    err = mqtt_client_connect(g_p_client, &g_broker_addr, MQTT_BROKER_PORT,
                              connection_cb, NULL, &client_info);
    if (err != ERR_OK)
    {
        LOG_ERR(TAG_S1_MQTT, "mqtt_client_connect() failed, err=%d", (int)err);
        g_connecting = 1;
        sys_untimeout(retry_timeout, NULL);
        sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
    }
}

#endif /* TM_MQTT */
