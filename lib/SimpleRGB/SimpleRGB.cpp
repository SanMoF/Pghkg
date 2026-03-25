#include "SimpleRGB.h"
#include "esp_timer.h" // esp_timer_get_time()
#include <cstdint>

SimpleRGB::SimpleRGB()
{
}

void SimpleRGB::setup(uint8_t pin[3], const uint8_t channel[3], TimerConfig *config, bool invert)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        pwm[i].setup(pin[i], channel[i], config, invert);
    }
}

void SimpleRGB::setColor(uint32_t red, uint32_t green, uint32_t blue)
{
    // stop blinking and force the color
    blink_active = false;
    blink_state_on = false;
    pwm[0].setDigitalLevel(red);
    pwm[1].setDigitalLevel(green);
    pwm[2].setDigitalLevel(blue);
}

void SimpleRGB::setColor(uint32_t color_code)
{
    setColor((color_code >> 16) & 0xFF, (color_code >> 8) & 0xFF, color_code & 0xFF);
}

void SimpleRGB::StopBlinkAndSet(uint32_t red, uint32_t green, uint32_t blue)
{
    setColor(red, green, blue);
}

void SimpleRGB::Blink(uint32_t red, uint32_t green, uint32_t blue, uint32_t period_us)
{
    uint64_t now = esp_timer_get_time();

    // If blink parameters changed or we weren't active, initialize
    if (!blink_active || blink_red != red || blink_green != green || blink_blue != blue || blink_period_us != period_us)
    {
        blink_active = true;
        blink_state_on = false; // so first call will turn it on
        blink_red = red;
        blink_green = green;
        blink_blue = blue;
        blink_period_us = (period_us == 0) ? 500000u : period_us; // avoid zero
        last_toggle_time_us = now;
    }

    // toggle when half-period elapsed (on/off each half)
    uint64_t half = (uint64_t)blink_period_us / 2u;
    if (now - last_toggle_time_us >= half)
    {
        last_toggle_time_us += half; // advance by half period to avoid drift
        blink_state_on = !blink_state_on;
        if (blink_state_on)
        {
            pwm[0].setDigitalLevel(blink_red);
            pwm[1].setDigitalLevel(blink_green);
            pwm[2].setDigitalLevel(blink_blue);
        }
        else
        {
            pwm[0].setDigitalLevel(0);
            pwm[1].setDigitalLevel(0);
            pwm[2].setDigitalLevel(0);
        }
    }
}
