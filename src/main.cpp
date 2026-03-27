#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "AS5600.h"
#include "SimpleGPIO.h"
#include "SimplePWM.h"
#include "esp_task_wdt.h"
#include "PID_CAYETANO.h"

#define DEADBAND_DEG 1.0f
#define TARGET_DEG 0.0f

static TimerConfig PWM_STEPPER_UP_TIMER{
    .timer = LEDC_TIMER_0,
    .frequency = 650,
    .bit_resolution = LEDC_TIMER_8_BIT,
    .mode = LEDC_LOW_SPEED_MODE};

extern "C" void app_main()
{
    esp_task_wdt_deinit();

    SimplePWM STEP;
    SimpleGPIO DIR;
    DIR.setup(5, GPO);
    STEP.setup(4, 0, &PWM_STEPPER_UP_TIMER);
    STEP.setDuty(0.0f);

    PID_CAYETANO PID;
    float gains[3] = {100, 0.1, 0};
    PID.setup(gains, 10000);
    PID.setULimit(2000);

    AS5600 Encoder;
    if (Encoder.begin() != ESP_OK)
    {
        printf("[ERROR] AS5600 not found – check wiring\n");
        return;
    }

    uint8_t status = 0;
    do
    {
        Encoder.getStatus(&status);
        printf("Waiting for magnet... STATUS=0x%02X\n", status);
        vTaskDelay(pdMS_TO_TICKS(300));
    } while (!(status & AS5600_STATUS_MD));

    printf("Magnet OK – homing started\n");

    while (1)
    {
        float angle = Encoder.getAccumulatedAngleDeg();

        if (isnan(angle))
        {
            printf("[ERROR] Read failed\n");
            STEP.setDuty(0.0f);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        float error = TARGET_DEG - angle;
        bool at_home = fabsf(error) < DEADBAND_DEG;
        float u = PID.computedU(error);

        printf("Angle: %6.2f  Error: %6.2f  U: %8.2f  %s\n",
               angle, error, u, at_home ? "HOME" : "MOVING");

        if (at_home)
        {
            STEP.setDuty(0.0f);
        }
        else
        {
            DIR.set(u > 0 ? 0 : 1);
            STEP.setDuty(50.0f);
            STEP.setFrequency(fabsf(u));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}