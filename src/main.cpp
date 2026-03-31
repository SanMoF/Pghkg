#include "definitions.h"

static void IRAM_ATTR timerinterrupt(void *arg)
{
    Timer.setInterrupt();
}

static void mqtt_task(void *pvParameters)
{
    while (1)
    {
        const char *raw = mqtt.subscribe();

        if (raw != nullptr && raw[0] != '\0')
        {
            char buf[64];
            strncpy(buf, raw, sizeof(buf) - 1);
            buf[sizeof(buf) - 1] = '\0';

            char *token = strtok(buf, ",");
            if (token == nullptr)
            {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            mode = atoi(token);

            if (mode == 0)
            {
                token = strtok(nullptr, ",");
                if (token)
                    theta1 = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    theta2 = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    theta3 = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    theta4 = atof(token);
            }
            else if (mode == 1)
            {
                token = strtok(nullptr, ",");
                if (token)
                    cmd_x = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    cmd_y = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    cmd_z = atof(token);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

extern "C" void app_main()
{
    esp_task_wdt_deinit();
    
    float gains[3] = {100, 1, 0};
    Timer.setup(timerinterrupt, "Main_timer");
    Timer.startPeriodic(dt);

    // motor.setup(5, 4, 0, &PWM_STEPPER_TIMER, gains, dt);
    wifi.setup(WIFI_SSID, WIFI_PASSWORD);
    mqtt.setup(MQTT_BROKER_URI, MQTT_CLIENT_ID, TOPIC_SUB);
    mqtt.publish(TOPIC_PUB, "ESP32 online");

    xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 5, NULL);

    while (1)
    {
        if (Timer.interruptAvailable())
        {
            switch (mode)
            {
            case 0:
                printf("Joints — t1:%.1f t2:%.1f t3:%.1f t4:%.1f\n",
                       theta1, theta2, theta3, theta4);
                break;

            case 1:
                printf("XYZ — x:%.1f y:%.1f z:%.1f\n",
                       cmd_x, cmd_y, cmd_z);
                break;

            default:
                break;
            }
        }
    }
}