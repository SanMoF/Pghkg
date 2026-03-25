#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mqtt_client.h"

// ─── Configuration ────────────────────────────────────────────────
#define WIFI_SSID "realme 11 Pro 5G"
#define WIFI_PASSWORD "z57ek35n"
#define MQTT_BROKER_URI "mqtt://10.48.251.213:1883"
#define MQTT_CLIENT_ID "ESP32_Client_01"

// Topics
#define TOPIC_PUB "esp32/status"
#define TOPIC_SUB "esp32/commands"

// ─── Globals ──────────────────────────────────────────────────────
static const char *TAG_WIFI = "WiFi";
static const char *TAG_MQTT = "MQTT";

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static esp_mqtt_client_handle_t mqtt_client = NULL;
static volatile bool mqtt_connected = false;
static int s_retry_num = 0;
#define WIFI_MAX_RETRY 5

// ─── WiFi Event Handler ───────────────────────────────────────────
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < WIFI_MAX_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG_WIFI, "Retrying connection... (%d/%d)", s_retry_num, WIFI_MAX_RETRY);
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG_WIFI, "Failed to connect to WiFi");
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG_WIFI, "Connected! IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ─── WiFi Init ────────────────────────────────────────────────────
static void wifi_init(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Wait until connected or failed
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG_WIFI, "WiFi connected successfully");
    }
    else
    {
        ESP_LOGE(TAG_WIFI, "WiFi connection failed");
    }
}

// ─── MQTT Event Handler ───────────────────────────────────────────
static void mqtt_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG_MQTT, "Connected to broker");
            mqtt_connected = true;
            esp_mqtt_client_subscribe(event->client, TOPIC_SUB, 0);
            ESP_LOGI(TAG_MQTT, "Subscribed to: %s", TOPIC_SUB);
            esp_mqtt_client_publish(event->client, TOPIC_PUB, "ESP32 online", 0, 0, 0);
            break;

        case MQTT_EVENT_DISCONNECTED:
            mqtt_connected = false;
            ESP_LOGW(TAG_MQTT, "Disconnected from broker");
            break;

        case MQTT_EVENT_DATA: {
            char topic[64]    = {0};
            char payload[128] = {0};
            snprintf(topic,   sizeof(topic),   "%.*s", event->topic_len, event->topic);
            snprintf(payload, sizeof(payload), "%.*s", event->data_len,  event->data);

            ESP_LOGI(TAG_MQTT, "Topic: %s | Payload: %s", topic, payload);

            if (strcmp(payload, "ping") == 0) {
                esp_mqtt_client_publish(event->client, TOPIC_PUB, "pong", 0, 0, 0);
                ESP_LOGI(TAG_MQTT, "Replied with pong");
            }
            break;
        }

        case MQTT_EVENT_SUBSCRIBED:
        case MQTT_EVENT_UNSUBSCRIBED:
        case MQTT_EVENT_PUBLISHED:
        case MQTT_EVENT_BEFORE_CONNECT:
        case MQTT_EVENT_DELETED:
        case MQTT_EVENT_ANY:
        case MQTT_USER_EVENT:
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG_MQTT, "MQTT error occurred");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGE(TAG_MQTT, "Last error: 0x%x",
                         event->error_handle->esp_tls_last_esp_err);
            }
            break;

        default:
            break;
    }
}

// ─── MQTT Init ────────────────────────────────────────────────────
static void mqtt_init(void)
{

    // ✅ C++ compatible
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.uri = MQTT_BROKER_URI;
    mqtt_cfg.credentials.client_id = MQTT_CLIENT_ID;

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    // ✅ Explicit cast
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(mqtt_client));
}

// ─── Publish Task ─────────────────────────────────────────────────
static void publish_task(void *pvParameters)
{
    int count = 0;
    char payload[64];

    while (1)
    {
        snprintf(payload, sizeof(payload), "heartbeat %d", count++);
        esp_mqtt_client_publish(mqtt_client, TOPIC_PUB, payload, 0, 0, 0);
        ESP_LOGI(TAG_MQTT, "Published: %s", payload);
        vTaskDelay(pdMS_TO_TICKS(5000)); // Every 5 seconds
    }
}

// ─── Entry Point ──────────────────────────────────────────────────
extern "C" void app_main(void) {
    // Init NVS (required for WiFi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();
    mqtt_init();

    xTaskCreate(publish_task, "publish_task", 4096, NULL, 5, NULL);
}