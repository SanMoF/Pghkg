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

    // Timers Setups
    Timer.setup(timerinterrupt, "Main_timer");
    Timer.startPeriodic(dt);
    // Motors Setup
    Base_Stepper.setup(Base_Dir_Pin, Base_PWM_Pin, Base_PWM_Ch,
                       &STEPPER_TIMER_0, base_gains, dt);

    Z_Stepper.setup(Z_Stepper_PWM_Pin, Z_Stepper_Dir_Pin, Z_Stepper_PWM_Ch,
                    &STEPPER_TIMER_1, /*steps_per_rev=*/1600,
                    z_gains[0], z_gains[1], z_gains[2], (uint32_t)dt);

    // Wifi and Mqtt setups
    wifi.setup(WIFI_SSID, WIFI_PASSWORD);
    mqtt.setup(MQTT_BROKER_URI, MQTT_CLIENT_ID, TOPIC_SUB);
    mqtt.publish(TOPIC_PUB, "ESP32 online");

    xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 5, NULL);
    float prev_theta2 = -9999.0f;

    while (1)
    {
        if (Timer.interruptAvailable())
        {
            switch (mode)
            {
                // local variable before while(1)

            case 0:
                Base_Stepper.goToAngle(theta1);
                Z_Stepper.goToAngle(theta2, 6000); // fixed base freq, no prev needed
                Z_Stepper.update();
                break;
            case 1:
                printf("XYZ — x:%.1f y:%.1f z:%.1f\n",
                       cmd_x, cmd_y, cmd_z);
                break;

            default:
                break;
            }

            float Base_position = Base_Stepper._encoder.getAccumulatedAngleDeg();
            snprintf(pub_buf, sizeof(pub_buf), "%.2f", Base_position);
            mqtt.publish(TOPIC_PUB, pub_buf);
        }
    }
}