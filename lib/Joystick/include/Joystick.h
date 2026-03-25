#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#include "SimpleADC.h"
#include "SimpleGPIO.h"
#include <cstdint>
#include "esp_timer.h"

class Joystick
{
public:
    Joystick();

    void setup(int pinx, int piny, int pinBtn);
    bool calibrate(uint64_t timeMicros);

    int readX();
    int readY();

    bool Up();
    bool Down();
    bool Left();
    bool Right();

    // Returns true if the joystick is in its neutral (centered) zone
    bool zero();

    // Prints current direction only if not centered; returns false if idle
    bool result();

    // Returns true once per button edge (using getEdge)
    bool Pressed();

private:
    SimpleADC _ejeX;
    SimpleADC _ejeY;
    SimpleGPIO _button;

    int _centerX;
    int _centerY;
    int _deadZoneX;
    int _deadZoneY;
    uint64_t _btn_last_change_us = 0;
bool _btn_last_state = false;      // nivel lógico anterior (raw)
bool _btn_stable_state = false;    // estado estabilizado (pressed true = plumbing)
const uint64_t _btn_debounce_us = 200000ULL; // 200 ms por defecto
};

#endif // __JOYSTICK_H__
