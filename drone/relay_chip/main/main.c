
#include <stdio.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi_types.h"

uint8_t peer[6];
bool paired = false;

uint8_t broadcast[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

void add_peer(uint8_t *mac) {
    esp_now_peer_info_t p = {0};
    memcpy(p.peer_addr, mac, 6);
    p.channel = 1;
    p.ifidx = WIFI_IF_STA;
    p.encrypt = false;
    esp_now_add_peer(&p);
}

void on_sent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    printf("TX -> %02X:%02X:%02X:%02X:%02X:%02X : %s\n",
        info->des_addr[0], info->des_addr[1], info->des_addr[2],
        info->des_addr[3], info->des_addr[4], info->des_addr[5],
        status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {

    printf("RX <- %02X:%02X:%02X:%02X:%02X:%02X : ",
        info->src_addr[0], info->src_addr[1], info->src_addr[2],
        info->src_addr[3], info->src_addr[4], info->src_addr[5]);

    fwrite(data, 1, len, stdout);
    printf("\n");

    if (len == 8 && memcmp(data, "DISCOVER", 8) == 0) {

        esp_now_peer_info_t p = {0};
        memcpy(p.peer_addr, info->src_addr, 6);
        p.channel = 1;
        p.ifidx = WIFI_IF_STA;
        p.encrypt = false;

        if (!esp_now_is_peer_exist(info->src_addr)) {
            esp_now_add_peer(&p);
        }

        esp_now_send(info->src_addr, (uint8_t*)"HERE", 4);
    }

    if (!paired && len == 4 && memcmp(data, "HERE", 4) == 0) {

        memcpy(peer, info->src_addr, 6);

        if (!esp_now_is_peer_exist(peer)) {
            add_peer(peer);
        }

        paired = true;

        printf("Paired with %02X:%02X:%02X:%02X:%02X:%02X\n",
            peer[0],peer[1],peer[2],peer[3],peer[4],peer[5]);
    }
}


void app_main(void) {
    nvs_flash_init();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();

    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    esp_now_init();
    esp_now_register_recv_cb(on_recv);
    esp_now_register_send_cb(on_sent);

    esp_now_peer_info_t b = {0};
    memset(b.peer_addr, 0xFF, 6);
    b.channel = 1;
    b.ifidx = WIFI_IF_STA;
    esp_now_add_peer(&b);

    while (1) {
        if (!paired) {
            esp_now_send(broadcast, (uint8_t*)"DISCOVER", 8);
            printf("Searching...\n");
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            char * msg = "hello from rc";
            int size = strlen(msg);
            char data[size + 2];
            data[0] = 0x21;
            data[1] = size + 2;
            for (int i = 0; i < size; ++i)
            {
                data[i + 2] = msg[i];
            }
            esp_now_send(peer, (uint8_t *)data, strlen(data));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}
