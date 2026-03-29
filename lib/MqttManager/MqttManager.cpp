#include "MqttManager.h"
#include <string.h>
#include "esp_log.h"

void MqttManager::setup(const char* broker_uri, const char* client_id, const char* topic_sub)
{
    _topic_sub = topic_sub;

    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri    = broker_uri;
    cfg.credentials.client_id = client_id;

    _client = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(
        _client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, _event_handler, this));
    ESP_ERROR_CHECK(esp_mqtt_client_start(_client));
}

void MqttManager::publish(const char* topic, const char* message)
{
    if (!_connected) return;
    esp_mqtt_client_publish(_client, topic, message, 0, 0, 0);
}

const char* MqttManager::subscribe()
{
    if (_last_msg[0] == '\0') return _last_msg;

    strncpy(_out_msg, _last_msg, sizeof(_out_msg) - 1);
    _out_msg[sizeof(_out_msg) - 1] = '\0';
    _last_msg[0] = '\0';
    return _out_msg;
}

void MqttManager::_event_handler(void* arg, esp_event_base_t base,
                                   int32_t event_id, void* event_data)
{
    MqttManager* self  = static_cast<MqttManager*>(arg);
    auto*        event = static_cast<esp_mqtt_event_handle_t>(event_data);

    switch ((esp_mqtt_event_id_t)event_id) {

        case MQTT_EVENT_CONNECTED:
            self->_connected = true;
            esp_mqtt_client_subscribe(event->client, self->_topic_sub, 0);
            ESP_LOGI(TAG, "Connected. Subscribed to: %s", self->_topic_sub);
            break;

        case MQTT_EVENT_DISCONNECTED:
            self->_connected = false;
            ESP_LOGW(TAG, "Disconnected");
            break;

        case MQTT_EVENT_DATA: {
            snprintf(self->_last_msg, sizeof(self->_last_msg),
                     "%.*s", event->data_len, event->data);

            // Strip trailing \r \n and spaces
            int len = strlen(self->_last_msg);
            while (len > 0 && (self->_last_msg[len-1] == '\n' ||
                                self->_last_msg[len-1] == '\r' ||
                                self->_last_msg[len-1] == ' ')) {
                self->_last_msg[--len] = '\0';
            }

            ESP_LOGI(TAG, "Received [len:%d]: %s", len, self->_last_msg);
            break;
        }

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "Error: 0x%x",
                     event->error_handle->esp_tls_last_esp_err);
            break;

        default:
            break;
    }
}