#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "driver/gpio.h"

#define TAG "NODE3"

#define RADAR_GPIO 4
#define DOOR_GPIO  5
#define LED_GPIO   8

#define CMD_HIJACK            0x03
#define CMD_MOTION_DETECTED   0x04
#define CMD_DOOR_TOUCH        0x05

typedef struct {
    uint8_t  command_id;
    uint8_t  node_id;
    uint32_t timestamp;
    uint8_t  payload[16];
    uint8_t  hmac[16];
} ahar_message_t;

static const uint8_t NODE1_MAC[6] = {0x1C, 0xDB, 0xD4, 0x3C, 0x40, 0xDC};
static const uint8_t NODE2_MAC[6] = {0xAC, 0xA7, 0x04, 0xBF, 0x0F, 0x44};

static uint32_t last_radar_time = 0;
static uint32_t last_door_time = 0;

static void init_gpio(void)
{
    gpio_config_t radar_conf = {
        .pin_bit_mask = (1ULL << RADAR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&radar_conf);

    gpio_config_t door_conf = {
        .pin_bit_mask = (1ULL << DOOR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&door_conf);

    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led_conf);
    gpio_set_level(LED_GPIO, 0);
}

static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
}

static void on_data_sent(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "Send failed");
    }
}

static void espnow_init(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_data_sent));

    esp_now_peer_info_t peer_info = {0};
    peer_info.channel = 1;
    peer_info.encrypt = false;

    memcpy(peer_info.peer_addr, NODE1_MAC, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));

    memcpy(peer_info.peer_addr, NODE2_MAC, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
}

static void send_command(uint8_t cmd, const uint8_t *dest_mac)
{
    ahar_message_t msg = {0};
    msg.command_id = cmd;
    msg.node_id = 3;
    msg.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    esp_err_t res = esp_now_send(dest_mac, (uint8_t *)&msg, sizeof(msg));
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "esp_now_send error: %s", esp_err_to_name(res));
    }
    ESP_LOGI(TAG, "Command 0x%02X sent", cmd);
}
void radar_task(void *arg)
{
    while (1) {
        if (gpio_get_level(RADAR_GPIO) == 1) {
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last_radar_time > 5000) {
                ESP_LOGI(TAG, "Motion detected -> Node 1");
                send_command(CMD_MOTION_DETECTED, NODE1_MAC);
                last_radar_time = now;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void door_task(void *arg)
{
    while (1) {
        if (gpio_get_level(DOOR_GPIO) == 0) { /* touch (active low) */
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last_door_time > 3000) {
                ESP_LOGI(TAG, "Door touch -> Hijack sent");
                send_command(CMD_HIJACK, NODE2_MAC);
                send_command(CMD_DOOR_TOUCH, NODE1_MAC);
                last_door_time = now;
                /* LED feedback */
                gpio_set_level(LED_GPIO, 1);
                vTaskDelay(pdMS_TO_TICKS(200));
                gpio_set_level(LED_GPIO, 0);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Node 3 Sensor Hub starting...");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_gpio();
    wifi_init();

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    ESP_LOGI(TAG, "WiFi MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    espnow_init();
    ESP_LOGI(TAG, "ESP-NOW initialized");

    ESP_LOGI(TAG, "Peers registered: Node 1, Node 2");

    xTaskCreate(radar_task, "radar_task", 4096, NULL, 5, NULL);
    xTaskCreate(door_task, "door_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Node 3 ready");
}

