#ifndef _BLDC_ESC_H_
#define _BLDC_ESC_H_

#include "SimplePWM.h"

// Unidirectional RC-ESC style driver: a 50 Hz PWM signal whose pulse width
// commands the ESC, same convention as a servo. 1000 us = idle/off, 2000 us
// = full throttle. No reverse — plain sensorless ESCs (e.g. plane/quad
// 30A ESCs) arm only at minimum pulse and have no concept of a braking or
// reverse zone below it, unlike a car-style bidirectional ESC. Built on top
// of SimplePWM (already-tested LEDC wrapper), same as the project's gripper
// Servo.
class BLDC_ESC
{
public:
    BLDC_ESC();

    // timer_config must already be set to 50 Hz — pass a TimerConfig shared
    // by both ESCs (they can share one LEDC timer, one channel each).
    void setup(uint8_t pin, uint8_t channel, TimerConfig *timer_config,
               uint16_t min_us = 1000, uint16_t max_us = 2000);

    // Holds the idle pulse (minimum). Most sensorless ESCs refuse to arm —
    // and will just beep an error — unless throttle is at true zero when
    // the battery is connected, so this must be the minimum pulse, not a
    // midpoint. Call once in app_main's setup (before the control loop
    // starts) and vTaskDelay after it; never call from the control loop.
    void arm();

    // pct in [0, 100]: 0 = idle, 100 = full throttle. Negative values are
    // clamped to 0 — this ESC has no reverse.
    void setThrottlePercent(float pct);

    void stop();

private:
    SimplePWM _pwm;
    uint16_t  _min_us;
    uint16_t  _max_us;
    uint32_t  _period_us;
};

#endif // _BLDC_ESC_H_
