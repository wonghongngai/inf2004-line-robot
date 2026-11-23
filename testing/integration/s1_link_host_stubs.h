/*
 * s1_link_host_stubs.h - INT-15 (int/bad_commands, #51): the kernel and lwIP
 * boundary of the REAL S1 receive path.
 *
 * Force-included after robot_host_stubs.h. robot_host_test.c, built with
 * S1_REAL_LINK=1, then includes the real mqtt_manager.c and subsystem1.c,
 * so a command travels exactly as on the robot:
 *   broker bytes -> incoming_publish_cb / incoming_data_cb (mqtt_manager.c)
 *   -> s1_deliver_command (subsystem1.c) -> command message buffer
 *   -> subsystem1_receive_command -> robot_control.c poll_commands (VC)
 *   -> motion_control.c (S2).
 * Only the message-buffer calls (tk_*_mbf) and the lwIP client calls are
 * faked, in robot_host_test.c. The lwIP headers mqtt_manager.c includes are
 * the one-line shims in s1_link_host/, which point back here.
 */
#ifndef S1_LINK_HOST_STUBS_H
#define S1_LINK_HOST_STUBS_H

#define S1_REAL_LINK 1
#define TM_MQTT 1

/* ---- micro T-Kernel message buffers (subsystem1.c) ---------------------- */
#define LOCAL static
#define TA_TFIFO 0x00000000
typedef unsigned int ATR;
typedef struct { void *exinf; ATR mbfatr; INT bufsz; INT maxmsz; } T_CMBF;
ID tk_cre_mbf(const T_CMBF *pk_cmbf);
ER tk_del_mbf(ID mbfid);
ER tk_snd_mbf(ID mbfid, const void *msg, INT msgsz, TMO tmout);
INT tk_rcv_mbf(ID mbfid, void *msg, TMO tmout);

/* ---- lwIP types and MQTT client API (mqtt_manager.c) -------------------- */
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef signed char err_t;
typedef uintptr_t mem_ptr_t;
#define ERR_OK 0
#define MQTT_DATA_FLAG_LAST 1
typedef struct { u32_t addr; } ip4_addr_t;
typedef struct { ip4_addr_t ip4; u8_t type; } ip_addr_t;
#define ip_2_ip4(ip) (&(ip)->ip4)
#define IPADDR_TYPE_V4 0U
#define IP_SET_TYPE(ip, t) ((ip)->type = (u8_t)(t))
int ip4addr_aton(const char *cp, ip4_addr_t *addr);

typedef struct mqtt_client_s mqtt_client_t;
typedef enum { MQTT_CONNECT_ACCEPTED = 0, MQTT_CONNECT_DISCONNECTED = 256 }
    mqtt_connection_status_t;
struct mqtt_connect_client_info_t {
    const char *client_id, *client_user, *client_pass;
    u16_t keep_alive;
    const char *will_topic, *will_msg;
    u8_t will_msg_len, will_qos, will_retain;
};
typedef void (*mqtt_connection_cb_t)(mqtt_client_t *, void *, mqtt_connection_status_t);
typedef void (*mqtt_incoming_publish_cb_t)(void *, const char *, u32_t);
typedef void (*mqtt_incoming_data_cb_t)(void *, const u8_t *, u16_t, u8_t);
typedef void (*mqtt_request_cb_t)(void *, err_t);
mqtt_client_t *mqtt_client_new(void);
err_t mqtt_client_connect(mqtt_client_t *client, const ip_addr_t *ip, u16_t port,
                          mqtt_connection_cb_t cb, void *arg,
                          const struct mqtt_connect_client_info_t *info);
void mqtt_disconnect(mqtt_client_t *client);
u8_t mqtt_client_is_connected(mqtt_client_t *client);
void mqtt_set_inpub_callback(mqtt_client_t *client, mqtt_incoming_publish_cb_t pub,
                             mqtt_incoming_data_cb_t data, void *arg);
err_t mqtt_subscribe(mqtt_client_t *client, const char *topic, u8_t qos,
                     mqtt_request_cb_t cb, void *arg);
err_t mqtt_publish(mqtt_client_t *client, const char *topic, const void *payload,
                   u16_t len, u8_t qos, u8_t retain, mqtt_request_cb_t cb, void *arg);

typedef void (*sys_timeout_handler)(void *arg);
void sys_timeout(u32_t msecs, sys_timeout_handler handler, void *arg);
void sys_untimeout(sys_timeout_handler handler, void *arg);
u32_t sys_now(void);

/* ---- wifi_secrets.h: the template's MQTT values ------------------------- */
#define MQTT_BROKER_IP   "192.168.1.100"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID   "group17-robot"
#define MQTT_TOPIC       MQTT_CLIENT_ID "/status"

#endif /* S1_LINK_HOST_STUBS_H */
