#ifndef _HBRIDGE_H
#define _HBRIDGE_H

#include "SimplePWM.h"

// Hbridge.h
class HBridge
{
private:
    SimplePWM PWM_CCLKW;
    SimplePWM PWM_CLKW;

public:
    HBridge();
void setup(uint8_t pwm_pin[], uint8_t pwm_channel[], TimerConfig* motor_pwm_config);
    void setSpeed(float speed);
    void setStop();
};

#endif // _HBRIDGE_H