// ============================================================================
// Stepper.cpp
// ============================================================================
#include "Stepper.h"
#include <stdio.h>

Stepper::Stepper()
    : _timer_config(nullptr),
      _step_pin(0),
      _dir_pin(0),
      _pwm_channel(0),
      _current_frequency(0),
      _direction(true),
      _calculated_position(0),
      _last_update_time(0),
      _steps_per_revolution(200),
      _target_position(0),
      _is_moving(false)
{
}

Stepper::~Stepper()
{
    _stepperPWM.setDuty(0);
}

void Stepper::setup(uint8_t step_pin, uint8_t dir_pin, uint8_t pwm_channel, TimerConfig *timer_config, uint32_t steps_per_rev)
{
    _step_pin = step_pin;
    _dir_pin = dir_pin;
    _pwm_channel = pwm_channel;
    _timer_config = timer_config;
    _current_frequency = timer_config->frequency;
    _steps_per_revolution = steps_per_rev;
    
    // Initialize PWM for STEP signal
    _stepperPWM.setup(_step_pin, _pwm_channel, _timer_config, false);
    _stepperPWM.setDuty(0);
    
    // Initialize DIR pin
    _dirGPIO.setup(_dir_pin, GPIO_MODE_OUTPUT, GPIO_FLOATING);
    _dirGPIO.set(_direction ? 1 : 0);
    
    _last_update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    
}

void Stepper::moveDegrees(float degrees, uint32_t frequency)
{
    // Update position before starting new move
    if (_is_moving)
    {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t elapsed_ms = current_time - _last_update_time;
        int32_t steps_moved = (_current_frequency * elapsed_ms) / 1000;
        
        if (_direction)
            _calculated_position += steps_moved;
        else
            _calculated_position -= steps_moved;
    }
    
    // Convert degrees to steps
    int32_t steps = (int32_t)((degrees / 360.0f) * _steps_per_revolution);
    _target_position = _calculated_position + steps;
    
    // Set direction
    _direction = (steps >= 0);
    _dirGPIO.set(_direction ? 1 : 0);
    
    // Set frequency
    _current_frequency = frequency;
    _stepperPWM.setFrequency(frequency);
    
    // Start movement
    _stepperPWM.setDuty(50.0f);
    _is_moving = true;
    _last_update_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    
}

int32_t Stepper::getPosition()
{
    if (_is_moving)
    {
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        uint32_t elapsed_ms = current_time - _last_update_time;
        
        int32_t steps_moved = (_current_frequency * elapsed_ms) / 1000;
        
        int32_t new_position;
        if (_direction)
            new_position = _calculated_position + steps_moved;
        else
            new_position = _calculated_position - steps_moved;
        
        // Check if target reached
        if ((_direction && new_position >= _target_position) ||
            (!_direction && new_position <= _target_position))
        {
            _calculated_position = _target_position;
            _stepperPWM.setDuty(0);
            _is_moving = false;
        }
        else
        {
            _calculated_position = new_position;
            _last_update_time = current_time;
        }
    }
    
    return _calculated_position;
}
void Stepper::forceStop()
{
    _stepperPWM.setDuty(0);     // stop pulses immediately
    _is_moving = false;         // stop non-blocking motion state
    _target_position = _calculated_position; 
}

