#pragma once

#include "mqtt_client.h"

class MqttManager {
public:
    void        setup(const char* broker_uri, const char* client_id, const char* topic_sub);
    void        publish(const char* topic, const char* message);
    const char* subscribe();

private:
    static void _event_handler(void* arg, esp_event_base_t base,
                                int32_t event_id, void* event_data);

    esp_mqtt_client_handle_t _client        = nullptr;
    const char*              _topic_sub     = nullptr;
    char                     _last_msg[256] = {0};
    char                     _out_msg[256]  = {0};
    volatile bool            _connected     = false;

    static constexpr const char* TAG = "MqttManager";
};