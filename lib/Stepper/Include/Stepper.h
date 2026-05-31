#pragma once
#include <stdint.h>
#include "SimplePWM.h"
#include "SimpleGPIO.h"
#include "PID_CAYETANO.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

struct TimerConfig;

class Stepper
{
public:
    // Public for debug printing
    int32_t _target_position;
    uint32_t _current_frequency;

    Stepper();
    ~Stepper();

    void setup(uint8_t step_pin, uint8_t dir_pin, uint8_t pwm_channel,
               TimerConfig *timer_config, uint32_t steps_per_rev,
               float kp, float ki, float kd, uint32_t dt_us);

    void goToAngle(float target_deg, uint32_t base_frequency); // absolute
    void moveDegrees(float degrees, uint32_t base_frequency);  // relative

    void update(); // call every timer tick

    int32_t getPosition() const;
    uint32_t stepsPerRev() const { return _steps_per_revolution; }
    bool isMoving() const { return _is_moving; }
    void forceStop();
    void resetPosition(); // zero step counter after homing

    void setPIDGains(float kp, float ki, float kd);
    void resetPID();

private:
    SimplePWM _stepperPWM;
    SimpleGPIO _dirGPIO;
    PID_CAYETANO _pid;

    TimerConfig *_timer_config;
    uint8_t _step_pin;
    uint8_t _dir_pin;
    uint8_t _pwm_channel;

    uint32_t _base_frequency;
    uint32_t _dt_us;

    bool _direction;
    int32_t _calculated_position;
    uint32_t _steps_per_revolution;

    uint32_t _last_update_ms;
    bool _is_moving;

    template <typename T>
    static T _clamp(T val, T lo, T hi)
    {
        return val < lo ? lo : (val > hi ? hi : val);
    }

    void _applyFrequency(uint32_t freq);
    int32_t _estimateNewPosition() const;
};