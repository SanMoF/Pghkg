#include "BLDC_ESC.h"

BLDC_ESC::BLDC_ESC()
    : _min_us(1000), _max_us(2000), _period_us(20000)
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

void BLDC_ESC::setThrottlePercent(float pct)
{
    if (pct < 0.0f)
        pct = 0.0f;
    if (pct > 100.0f)
        pct = 100.0f;

    float pulse_us = (float)_min_us + (pct / 100.0f) * (float)(_max_us - _min_us);

    float duty_percent = (pulse_us * 100.0f) / (float)_period_us;
    _pwm.setDuty(duty_percent);
}

void BLDC_ESC::stop()
{
    setThrottlePercent(0.0f);
}
