#include "ServoStepper.h"

static TimerConfig PWM_STEPPER_TIMER{
    .timer          = LEDC_TIMER_0,
    .frequency      = 650,
    .bit_resolution = LEDC_TIMER_8_BIT,
    .mode           = LEDC_LOW_SPEED_MODE
};

extern "C" void app_main()
{
    ServoStepper motor;
    float gains[3] = {100, 0.1, 0};
    motor.setup(5, 4, 0, &PWM_STEPPER_TIMER, gains, 10000);

    while (1) {
        motor.goToAngle(90.0f);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}