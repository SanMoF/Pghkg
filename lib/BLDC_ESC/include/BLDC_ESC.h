#ifndef _BLDC_ESC_H_
#define _BLDC_ESC_H_

#include "SimplePWM.h"

// Bidirectional RC-ESC style driver: a 50 Hz PWM signal whose pulse width
// commands the ESC, same convention as a servo. 1500 us = stop, 1000 us =
// full reverse, 2000 us = full throttle forward. Built on top of SimplePWM
// (already-tested LEDC wrapper), same as the project's gripper Servo.
class BLDC_ESC
{
public:
    BLDC_ESC();

    // timer_config must already be set to 50 Hz — pass a TimerConfig shared
    // by both ESCs (they can share one LEDC timer, one channel each).
    void setup(uint8_t pin, uint8_t channel, TimerConfig *timer_config,
               uint16_t min_us = 1000, uint16_t max_us = 2000);

    // Holds the stop pulse. Most ESCs need this held for ~2 s to arm —
    // call once in app_main's setup (before the control loop starts) and
    // vTaskDelay after it; never call from the control loop.
    void arm();

    // pct in [-100, 100]: negative = reverse, 0 = stop, positive = forward.
    void setThrottlePercent(float pct);

    void stop();

private:
    SimplePWM _pwm;
    uint16_t  _min_us;
    uint16_t  _max_us;
    uint16_t  _center_us;
    uint32_t  _period_us;
};

#endif // _BLDC_ESC_H_
