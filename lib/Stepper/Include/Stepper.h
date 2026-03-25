// ============================================================================
// Stepper.h
// ============================================================================
#ifndef STEPPER_H
#define STEPPER_H

#include "SimplePWM.h"
#include "SimpleGPIO.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class Stepper
{
private:
    SimplePWM _stepperPWM;
    SimpleGPIO _dirGPIO;
    TimerConfig *_timer_config;
    
    uint8_t _step_pin;
    uint8_t _dir_pin;
    uint8_t _pwm_channel;
    uint32_t _current_frequency;
    bool _direction;
    
    int32_t _calculated_position;
    uint32_t _last_update_time;
    uint32_t _steps_per_revolution;
    
    int32_t _target_position;
    bool _is_moving;

public:
    Stepper();
    ~Stepper();
    
    void setup(uint8_t step_pin, uint8_t dir_pin, uint8_t pwm_channel, TimerConfig *timer_config, uint32_t steps_per_rev);
    void moveDegrees(float degrees, uint32_t frequency);
    void forceStop();

    int32_t getPosition();
};

#endif // STEPPER_H
