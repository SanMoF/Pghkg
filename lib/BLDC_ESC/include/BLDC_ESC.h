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
    // clamped to 0 — this ESC has no reverse. Below deadbandPercent (see
    // setDeadbandPercent), any pct > 0 is remapped to start right at the
    // motor's real spin-up threshold instead of wasting the low end of the
    // range on a pulse too weak to turn the rotor.
    void setThrottlePercent(float pct);

    // Calibrates out the ESC/motor's spin-up dead zone. `pct` is the lowest
    // commanded percent (0-100, on the old linear min-to-max scale) at
    // which the motor was observed to actually start turning — found by
    // ramping setThrottlePercent() up from 0 until it moves. Once set,
    // requesting any pct in (0, 100] rescales onto [pct, 100] instead of
    // [0, 100], so 1% already spins the motor — giving the PID usable
    // resolution near the start of the range instead of a dead first
    // stretch. 0% still means true idle. Defaults to 0 (no rescaling).
    void setDeadbandPercent(float pct);

    // Caps how far setThrottlePercent() is allowed to go, e.g. 30 limits the
    // motor to 30% even if a caller asks for 100. Runtime-adjustable (unlike
    // the deadband, which is a one-time calibration) so it can be tuned from
    // the HMI while the motor is spinning. Defaults to 100 (no cap).
    void setMaxThrottlePercent(float pct);

    void stop();

private:
    SimplePWM _pwm;
    uint16_t  _min_us;
    uint16_t  _max_us;
    uint32_t  _period_us;
    float     _deadband_pct;
    float     _max_throttle_pct;
};

#endif // _BLDC_ESC_H_
