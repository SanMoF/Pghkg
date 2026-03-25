#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "AS5600.h"
#include "SimpleGPIO.h"
#include "SimplePWM.h"
#include "esp_task_wdt.h"

#define DEADBAND_DEG  2.0f

static TimerConfig PWM_STEPPER_UP_TIMER {
    .timer          = LEDC_TIMER_0,
    .frequency      = 650,
    .bit_resolution = LEDC_TIMER_8_BIT,
    .mode           = LEDC_LOW_SPEED_MODE
};

extern "C" void app_main()
{
    esp_task_wdt_deinit();

    SimplePWM  STEP;
    SimpleGPIO DIR;
    DIR.setup(5, GPO);
    STEP.setup(4, 0, &PWM_STEPPER_UP_TIMER);
    STEP.setDuty(0.0f);   // motor OFF until we know where we are

    AS5600 Encoder;
    if (Encoder.begin() != ESP_OK) {
        printf("[ERROR] AS5600 not found – check wiring\n");
        return;
    }

    // Wait for magnet (MD bit only, ignores field strength)
    uint8_t status = 0;
    do {
        Encoder.getStatus(&status);
        printf("Waiting for magnet... STATUS=0x%02X\n", status);
        vTaskDelay(pdMS_TO_TICKS(300));
    } while (!(status & AS5600_STATUS_MD));

    printf("Magnet OK – homing started\n");

    while (1) {
        float angle = Encoder.getAngleDeg();

        if (isnan(angle)) {
            printf("[ERROR] Read failed\n");
            STEP.setDuty(0.0f);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Signed shortest-path error toward 0°
        float error   = (angle > 180.0f) ? angle - 360.0f : angle;
        bool  at_home = fabsf(error) < DEADBAND_DEG;

        printf("Angle: %6.2f  Error: %6.2f  %s\n",
               angle, error, at_home ? "HOME" : "MOVING");

        if (!at_home) {
            // angle <= 180 → go CW (DIR=1), angle > 180 → go CCW (DIR=0)
            DIR.set(angle <= 180.0f ? 1 : 0);
            STEP.setDuty(50.0f);
            STEP.setFrequency(2000);
        } else {
            STEP.setDuty(0.0f);   // stop at home
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}