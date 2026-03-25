#include "Hbridge.h"

HBridge::HBridge() {}

void HBridge::setup(uint8_t pwm_pin[], uint8_t pwm_channel[], TimerConfig* motor_pwm_config)
{
    PWM_CCLKW.setup(pwm_pin[0], pwm_channel[0], motor_pwm_config);
    PWM_CLKW.setup(pwm_pin[1], pwm_channel[1], motor_pwm_config);
}

void HBridge::setSpeed(float speed)
{
    if (speed > 100.0f) speed = 100.0f;
    if (speed < -100.0f) speed = -100.0f;

    if (speed > 0.0f)
    {
        PWM_CLKW.setDuty(speed);
        PWM_CCLKW.setDuty(0.0f);
    }
    else
    {
        PWM_CLKW.setDuty(0.0f);
        PWM_CCLKW.setDuty(-speed);
    }
}

void HBridge::setStop()
{
    PWM_CLKW.setDuty(0.0f);
    PWM_CCLKW.setDuty(0.0f);
}
