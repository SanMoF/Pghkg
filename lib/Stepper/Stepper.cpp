#include "Stepper.h"
#include <stdio.h>
#include <math.h>

Stepper::Stepper()
    : _target_position(0),
      _current_frequency(0),
      _timer_config(nullptr),
      _step_pin(0),
      _dir_pin(0),
      _pwm_channel(0),
      _base_frequency(0),
      _dt_us(10000),
      _direction(true),
      _calculated_position(0),
      _steps_per_revolution(200),
      _last_update_ms(0),
      _is_moving(false)
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
    _pid.setULimit(4000.0f);
    _pid.reset();

    _last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

// ── goToAngle — absolute target, rearms only on new target ───────────────────
void Stepper::goToAngle(float target_deg, uint32_t base_frequency)
{
    int32_t target_steps = (int32_t)((target_deg / 360.0f) * (float)_steps_per_revolution);

    if (target_steps == _target_position) return;  // same target, do nothing

    _target_position = target_steps;
    _base_frequency  = base_frequency;
    _direction       = (_target_position > _calculated_position);
    _dirGPIO.set(_direction ? 1 : 0);

    _pid.reset();
    _applyFrequency(_base_frequency);
    _stepperPWM.setDuty(50.0f);
    _is_moving      = true;
    _last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

// ── moveDegrees — relative move from current position ────────────────────────
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

// ── update — call every timer tick ───────────────────────────────────────────
void Stepper::update()
{
    if (!_is_moving) return;

    _calculated_position = _estimateNewPosition();
    _last_update_ms      = xTaskGetTickCount() * portTICK_PERIOD_MS;

    bool reached = (_direction  && _calculated_position >= _target_position) ||
                   (!_direction && _calculated_position <= _target_position);

    if (reached)
    {
        _calculated_position = _target_position;  // hard snap — kills drift
        _stepperPWM.setDuty(0);
        _is_moving = false;
        _pid.reset();
        return;
    }

    // PID: error = remaining steps, output = frequency
    int32_t remaining  = _target_position - _calculated_position;
    float   error      = (float)remaining;
    float   freq       = fabsf(_pid.computedU(error));

    // Update direction in case PID overshoots
    bool new_dir = (remaining > 0);
    if (new_dir != _direction)
    {
        _direction = new_dir;
        _dirGPIO.set(_direction ? 1 : 0);
    }

    if (freq < 300.0f)
        _stepperPWM.setDuty(0.0f);  // below LEDC minimum, coast
    else
        _applyFrequency((uint32_t)_clamp((int32_t)freq, (int32_t)300, (int32_t)16000));
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