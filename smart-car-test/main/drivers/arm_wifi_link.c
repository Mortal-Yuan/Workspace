#include "arm_link.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

/* Owned by controller task; socket I/O after initialization never blocks. */
static int listener = -1, peer = -1;
static int64_t last_heartbeat, last_received;
static bool lost, healthy;
static uint16_t last_sequence;

static void disconnect_peer(arm_link_t *link)
{
    if (peer >= 0) { close(peer); peer = -1; lost = true; }
    healthy = false;
    link->receive_length = 0;
}

static bool send_frame(arm_link_t *link, const char *frame, size_t length)
{
    if (peer < 0) return false;
    if (send(peer, frame, length, 0) == (int)length) return true;
    /* Never continue a partial command on a new connection. */
    disconnect_peer(link);
    return false;
}

esp_err_t arm_link_init(arm_link_t *link, int tx_pin, int rx_pin)
{
    (void)tx_pin; (void)rx_pin;
    if (!link) return ESP_ERR_INVALID_ARG;
    memset(link, 0, sizeof(*link));
    esp_err_t err = nvs_flash_init();
    if (err != ESP_OK) return err;
    if ((err = esp_netif_init()) != ESP_OK) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    if (!esp_netif_create_default_wifi_ap()) return ESP_ERR_NO_MEM;
    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    if ((err = esp_wifi_init(&init)) != ESP_OK) return err;
    wifi_config_t config = {.ap = {
        .ssid = "CarArm-8F9C", .password = "ArmLink-2026-8F9C",
        .channel = 6, .authmode = WIFI_AUTH_WPA2_PSK, .max_connection = 1,
    }};
    if ((err = esp_wifi_set_mode(WIFI_MODE_AP)) != ESP_OK) return err;
    if ((err = esp_wifi_set_config(WIFI_IF_AP, &config)) != ESP_OK) return err;
    if ((err = esp_wifi_start()) != ESP_OK) return err;
    listener = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listener < 0) return ESP_FAIL;
    struct sockaddr_in addr = {.sin_family = AF_INET,
        .sin_port = htons(8266), .sin_addr.s_addr = htonl(INADDR_ANY)};
    if (fcntl(listener, F_SETFL, O_NONBLOCK) < 0 ||
        bind(listener, (struct sockaddr *)&addr, sizeof(addr)) < 0 ||
        listen(listener, 1) < 0) {
        close(listener); listener = -1; return ESP_FAIL;
    }
    link->next_sequence = 1;
    link->initialized = true;
    return ESP_OK;
}

esp_err_t arm_link_send_command(arm_link_t *link, const char *command, uint16_t *sequence)
{
    if (!link || !link->initialized || !healthy) return ESP_ERR_INVALID_STATE;
    const uint16_t current = link->next_sequence++;
    if (link->next_sequence > 9999) link->next_sequence = 1;
    char frame[32];
    int length = snprintf(frame, sizeof(frame), "CAR,%s,%04u\n", command, current);
    if (length <= 0 || length >= (int)sizeof(frame)) return ESP_ERR_INVALID_SIZE;
    if (!send_frame(link, frame, length)) return ESP_FAIL;
    last_sequence = current;
    if (sequence) *sequence = current;
    return ESP_OK;
}

bool arm_link_poll(arm_link_t *link, arm_link_event_t *event)
{
    if (!link || !event || !link->initialized) return false;
    *event = (arm_link_event_t){0};
    int64_t now = esp_timer_get_time();
    if (peer >= 0 && now - last_received > 2000000) disconnect_peer(link);
    if (lost) {
        lost = false;
        event->kind = ARM_LINK_EVENT_ERROR;
        event->sequence = last_sequence;
        strcpy(event->line, "WIFI_DISCONNECTED");
        return true;
    }
    if (peer < 0) {
        peer = accept(listener, NULL, NULL);
        if (peer < 0) return false;
        if (fcntl(peer, F_SETFL, O_NONBLOCK) < 0) {
            disconnect_peer(link); return false;
        }
        last_received = now; last_heartbeat = 0;
        link->receive_length = 0; healthy = false;
    }
    if (now - last_heartbeat >= 500000) {
        const char *heart = "CAR,HEART,0000\n";
        if (!send_frame(link, heart, strlen(heart))) return false;
        last_heartbeat = now;
    }
    for (int i = 0; i < 128; ++i) {
        char byte;
        int n = recv(peer, &byte, 1, 0);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
        if (n <= 0) { disconnect_peer(link); break; }
        if (byte == '\r') continue;
        if (byte == '\n') {
            link->receive_line[link->receive_length] = 0;
            link->receive_length = 0;
            if (strcmp(link->receive_line, "ARM,HEART,0000") == 0) {
                last_received = now; healthy = true; continue;
            }
            char kind[12], extra; unsigned sequence;
            if (sscanf(link->receive_line, "ARM,%11[^,],%4u%c", kind, &sequence, &extra) != 2 ||
                sequence == 0 || sequence > 9999) continue;
            event->kind = strcmp(kind,"PONG")==0 ? ARM_LINK_EVENT_PONG :
                strcmp(kind,"ACK")==0 ? ARM_LINK_EVENT_ACK :
                strcmp(kind,"DONE")==0 ? ARM_LINK_EVENT_DONE :
                strcmp(kind,"BUSY")==0 ? ARM_LINK_EVENT_BUSY :
                strcmp(kind,"ERROR")==0 ? ARM_LINK_EVENT_ERROR : ARM_LINK_EVENT_UNKNOWN;
            event->sequence = sequence;
            strcpy(event->line, link->receive_line);
            return true;
        }
        if (link->receive_length + 1 >= sizeof(link->receive_line)) {
            disconnect_peer(link); break;
        }
        link->receive_line[link->receive_length++] = byte;
    }
    return false;
}

esp_err_t arm_link_send_ping(arm_link_t *link, uint16_t *sequence)
{ return arm_link_send_command(link, "PING", sequence); }
