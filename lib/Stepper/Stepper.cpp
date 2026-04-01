#include "Stepper.h"
#include <stdio.h>

Stepper::Stepper()
    : _timer_config(nullptr),
      _step_pin(0), _dir_pin(0), _pwm_channel(0),
      _base_frequency(0), _current_frequency(0), _dt_us(10000),
      _direction(true),
      _calculated_position(0), _target_position(0),
      _steps_per_revolution(200),
      _last_update_ms(0), _is_moving(false)
{}

Stepper::~Stepper()
{
    _stepperPWM.setDuty(0);
}

void Stepper::setup(uint8_t step_pin, uint8_t dir_pin, uint8_t pwm_channel,
                    TimerConfig *timer_config, uint32_t steps_per_rev,
                    float kp, float ki, float kd, uint32_t dt_us)
{
    _step_pin             = step_pin;
    _dir_pin              = dir_pin;
    _pwm_channel          = pwm_channel;
    _timer_config         = timer_config;
    _steps_per_revolution = steps_per_rev;
    _dt_us                = dt_us;

    _stepperPWM.setup(_step_pin, _pwm_channel, _timer_config, false);
    _stepperPWM.setDuty(0);

    _dirGPIO.setup(_dir_pin, GPIO_MODE_OUTPUT, GPIO_FLOATING);
    _dirGPIO.set(1);

    float gains[3] = {kp, ki, kd};
    _pid.setup(gains, (float)dt_us);
    _pid.setULimit(500.0f);
    _pid.reset();

    _last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void Stepper::moveDegrees(float degrees, uint32_t base_frequency)
{
    if (_is_moving)
        _calculated_position = _estimateNewPosition();

    int32_t steps    = (int32_t)((degrees / 360.0f) * (float)_steps_per_revolution);
    _target_position = _calculated_position + steps;
    _base_frequency  = base_frequency;

    _direction = (steps >= 0);
    _dirGPIO.set(_direction ? 1 : 0);

    _pid.reset();
    _applyFrequency(_base_frequency);
    _stepperPWM.setDuty(50.0f);

    _is_moving      = true;
    _last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

void Stepper::update()
{
    if (!_is_moving) return;

    _calculated_position = _estimateNewPosition();
    _last_update_ms      = xTaskGetTickCount() * portTICK_PERIOD_MS;

    bool reached = (_direction  && _calculated_position >= _target_position) ||
                   (!_direction && _calculated_position <= _target_position);

    if (reached)
    {
        _calculated_position = _target_position;
        _stepperPWM.setDuty(0);
        _is_moving = false;
        _pid.reset();
        return;
    }

    float   error      = (float)(_target_position - _calculated_position);
    float   correction = _pid.computedU(error);
    int32_t new_freq   = (int32_t)_base_frequency + (int32_t)correction;

    _applyFrequency(_clamp(new_freq, (int32_t)1, (int32_t)20000));
}

int32_t Stepper::getPosition() const
{
    return _calculated_position;
}

void Stepper::forceStop()
{
    _stepperPWM.setDuty(0);
    _is_moving       = false;
    _target_position = _calculated_position;
    _pid.reset();
}

void Stepper::setPIDGains(float kp, float ki, float kd)
{
    float gains[3] = {kp, ki, kd};
    _pid.setup(gains, (float)_dt_us);
}

void Stepper::resetPID()
{
    _pid.reset();
}

void Stepper::_applyFrequency(uint32_t freq)
{
    _current_frequency = freq;
    _stepperPWM.setFrequency(freq);
}

int32_t Stepper::_estimateNewPosition() const
{
    uint32_t now_ms  = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = now_ms - _last_update_ms;
    int32_t  steps   = (int32_t)((_current_frequency * elapsed) / 1000);
    return _direction ? _calculated_position + steps
                      : _calculated_position - steps;
}