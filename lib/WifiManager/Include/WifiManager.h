#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

class WifiManager {
public:
    void        setup(const char* ssid, const char* password, int max_retry = 5);
    bool        isConnected() const { return _connected; }

private:
    static void _event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);

    EventGroupHandle_t _event_group  = nullptr;
    volatile bool      _connected    = false;
    int                _retry_num    = 0;
    int                _max_retry    = 5;

    static constexpr const char* TAG = "WifiManager";

    #define WIFI_CONNECTED_BIT BIT0
    #define WIFI_FAIL_BIT      BIT1
};