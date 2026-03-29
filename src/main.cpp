#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "WifiManager.h"
#include "MqttManager.h"

// ─── Configuration ────────────────────────────────────────────────
#define WIFI_SSID       "WIFI_RAY"
#define WIFI_PASSWORD   "Santi2011"
#define MQTT_BROKER_URI "mqtt://192.168.80.158:1883"
#define MQTT_CLIENT_ID  "ESP32_Client_01"
#define TOPIC_PUB       "esp32/status"
#define TOPIC_SUB       "esp32/commands"

static const char* TAG = "main";

// ─── State machine ────────────────────────────────────────────────
typedef enum {
    STATE_IDLE,
    STATE_HOMING,
    STATE_MOVING,
    STATE_STOP,
    STATE_ERROR
} RobotState;

// ─── Shared variables (written by MQTT task, read by main loop) ───
static volatile RobotState robot_state = STATE_IDLE;
static volatile float      target_x    = 0.0f;
static volatile float      target_y    = 0.0f;

// ─── Managers ─────────────────────────────────────────────────────
WifiManager wifi;
MqttManager mqtt;

// ─── MQTT task: parses commands and updates shared variables ──────
static void mqtt_task(void* pvParameters)
{
    char payload[64];

    while (1) {
        const char* cmd = mqtt.subscribe();

        if (cmd[0] != '\0') {
            ESP_LOGI(TAG, "Command received: %s", cmd);

            // ── State transitions ──────────────────────────────────
            if      (strcmp(cmd, "IDLE")  == 0) robot_state = STATE_IDLE;
            else if (strcmp(cmd, "HOME")  == 0) robot_state = STATE_HOMING;
            else if (strcmp(cmd, "STOP")  == 0) robot_state = STATE_STOP;
            else if (strcmp(cmd, "ERROR") == 0) robot_state = STATE_ERROR;

            // ── Variable updates (format: "MOVE x y") ─────────────
            else if (strncmp(cmd, "MOVE", 4) == 0) {
                float x, y;
                if (sscanf(cmd, "MOVE %f %f", &x, &y) == 2) {
                    target_x    = x;
                    target_y    = y;
                    robot_state = STATE_MOVING;
                    ESP_LOGI(TAG, "Target -> x:%.2f y:%.2f", target_x, target_y);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // poll every 50ms
    }
}

// ─── Entry Point ──────────────────────────────────────────────────
extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi.setup(WIFI_SSID, WIFI_PASSWORD);
    mqtt.setup(MQTT_BROKER_URI, MQTT_CLIENT_ID, TOPIC_SUB);
    mqtt.publish(TOPIC_PUB, "ESP32 online");

    // MQTT task runs separately — does not block the main loop
    xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 5, NULL);

    // ─── Main loop: state machine ─────────────────────────────────
    char status[64];

RobotState last_state = STATE_IDLE;  // antes del while(1)

while (1) {
    switch (robot_state) {
        case STATE_IDLE:
            snprintf(status, sizeof(status), "La ESP dice: IDLE");
            break;
        case STATE_HOMING:
            snprintf(status, sizeof(status), "La ESP dice: HOMING");
            break;
        case STATE_MOVING:
            snprintf(status, sizeof(status), "La ESP dice: MOVING x:%.2f y:%.2f", target_x, target_y);
            break;
        case STATE_STOP:
            snprintf(status, sizeof(status), "La ESP dice: STOP");
            break;
        case STATE_ERROR:
            snprintf(status, sizeof(status), "La ESP dice: ERROR");
            break;
    }

    if (robot_state != last_state) {
        mqtt.publish(TOPIC_PUB, status);
        last_state = robot_state;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
}

}