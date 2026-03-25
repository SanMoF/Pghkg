#ifndef __SIMPLERGB_H__
#define __SIMPLERGB_H__

#include "SimplePWM.h"
#include <cstdint>

class SimpleRGB
{
public:
    SimpleRGB();

    void setup(uint8_t pin[3], const uint8_t channel[3], TimerConfig *config, bool invert);

    // immediate set (also disables any ongoing blink)
    void setColor(uint32_t red, uint32_t green, uint32_t blue);
    void setColor(uint32_t color_code);

    // non-blocking blink: call this repeatedly from your main loop or tick handler.
    // period_us: full on-off period in microseconds (default 500000 = 500 ms)
    // Example: Blink(255,255,0,500000) -> yellow blink 0.5s on / 0.5s off
    void Blink(uint32_t red, uint32_t green, uint32_t blue, uint32_t period_us = 500000);

    // optional helper to stop blinking and set the color
    void StopBlinkAndSet(uint32_t red, uint32_t green, uint32_t blue);

private:
    SimplePWM pwm[3];

    // blink state
    bool blink_active = false;
    bool blink_state_on = false;      // current on/off state
    uint32_t blink_red = 0;
    uint32_t blink_green = 0;
    uint32_t blink_blue = 0;
    uint32_t blink_period_us = 0;     // full on-off period
    uint64_t last_toggle_time_us = 0; // last toggle timestamp
};

#endif // __SIMPLERGB_H__
