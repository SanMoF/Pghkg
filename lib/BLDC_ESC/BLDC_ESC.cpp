#include "BLDC_ESC.h"

BLDC_ESC::BLDC_ESC()
    : _min_us(1000), _max_us(2000), _period_us(20000), _deadband_pct(0.0f),
      _max_throttle_pct(100.0f)
{
}

void BLDC_ESC::setup(uint8_t pin, uint8_t channel, TimerConfig *timer_config,
                      uint16_t min_us, uint16_t max_us)
{
    _min_us    = min_us;
    _max_us    = max_us;
    _period_us = 1000000UL / timer_config->frequency;

    _pwm.setup(pin, channel, timer_config, false);
    stop();
}

void BLDC_ESC::arm()
{
    stop();
}

void BLDC_ESC::setDeadbandPercent(float pct)
{
    if (pct < 0.0f)
        pct = 0.0f;
    if (pct > 99.0f) // must leave room for the rescale below to reach 100%
        pct = 99.0f;
    _deadband_pct = pct;
}

void BLDC_ESC::setMaxThrottlePercent(float pct)
{
    if (pct < 0.0f)
        pct = 0.0f;
    if (pct > 100.0f)
        pct = 100.0f;
    _max_throttle_pct = pct;
}

void BLDC_ESC::setThrottlePercent(float pct)
{
    if (pct < 0.0f)
        pct = 0.0f;
    if (pct > _max_throttle_pct)
        pct = _max_throttle_pct;

    // Rescale (0, 100] onto [_deadband_pct, 100] so any positive request
    // clears the motor's real spin-up threshold instead of landing in the
    // dead zone below it. 0 stays true idle.
    float effective_pct = 0.0f;
    if (pct > 0.0f)
        effective_pct = _deadband_pct + (pct / 100.0f) * (100.0f - _deadband_pct);

    float pulse_us = (float)_min_us + (effective_pct / 100.0f) * (float)(_max_us - _min_us);

    float duty_percent = (pulse_us * 100.0f) / (float)_period_us;
    _pwm.setDuty(duty_percent);
}

void BLDC_ESC::stop()
{
    setThrottlePercent(0.0f);
}
