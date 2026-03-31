#include "WifiManager.h"
#include "esp_netif.h"
#include "nvs_flash.h"

void WifiManager::setup(const char* ssid, const char* password, int max_retry)
{
    _max_retry  = max_retry;
    _event_group = xEventGroupCreate();
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &_event_handler, this, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &_event_handler, this, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char*)wifi_config.sta.ssid,     ssid,     sizeof(wifi_config.sta.ssid)     - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Block until connected or failed
    EventBits_t bits = xEventGroupWaitBits(_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT)
        ESP_LOGI(TAG, "Connected successfully");
    else
        ESP_LOGE(TAG, "Connection failed");
}

void WifiManager::_event_handler(void* arg, esp_event_base_t event_base,
                                  int32_t event_id, void* event_data)
{
    WifiManager* self = static_cast<WifiManager*>(arg);

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        self->_connected = false;
        if (self->_retry_num < self->_max_retry) {
            esp_wifi_connect();
            self->_retry_num++;
            ESP_LOGW(TAG, "Retrying... (%d/%d)", self->_retry_num, self->_max_retry);
        } else {
            xEventGroupSetBits(self->_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Failed to connect");
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        ESP_LOGI(TAG, "IP: " IPSTR, IP2STR(&event->ip_info.ip));
        self->_retry_num = 0;
        self->_connected = true;
        xEventGroupSetBits(self->_event_group, WIFI_CONNECTED_BIT);
    }
}