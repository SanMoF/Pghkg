#include "Joystick.h"

Joystick::Joystick()
{
    _centerX = _centerY = 2048;
    _deadZoneX = _deadZoneY = 400;
}

void Joystick::setup(int pinx, int piny, int pinBtn)
{
    _ejeX.setup(pinx);
    _ejeY.setup(piny);
    _button.setup(pinBtn, GPIO_MODE_INPUT, GPIO_PULLUP_ONLY);
}

bool Joystick::calibrate(uint64_t timeMicros)
{
    int64_t current = 0, prev = 0;
    int64_t begin = 0;

    int minX = 4095, maxX = 0;
    int minY = 4095, maxY = 0;
    int sumX = 0, sumY = 0, samples = 0;

    prev = esp_timer_get_time();
    begin = esp_timer_get_time();
    int stop = begin + timeMicros;

    while (current < stop)
    {
        current = esp_timer_get_time();
        if (current - prev >= 1000)
        {
            int valX = readX();
            int valY = readY();

            sumX += valX;
            sumY += valY;

            if (valX < minX)
                minX = valX;
            if (valX > maxX)
                maxX = valX;

            if (valY < minY)
                minY = valY;
            if (valY > maxY)
                maxY = valY;

            prev = current;
            samples++;
        }
    }
    if (samples == 0)
        return false;
    _centerX = sumX / samples;
    _centerY = sumY / samples;

    _deadZoneX = (maxX - minX) * 0.90;
    _deadZoneY = (maxY - minY) * 1.05;

    printf("Calibration: %d %d %d %d\n", _centerX, _centerY, _deadZoneX, _deadZoneY);
    return true;
}

int Joystick::readX()
{
    return _ejeX.read(ADC_READ_RAW);
}

int Joystick::readY()
{
    return _ejeY.read(ADC_READ_RAW);
}

bool Joystick::Up()
{
    return readY() < (_centerY - _deadZoneY);
}

bool Joystick::Down()
{
    return readY() > (_centerY + _deadZoneY);
}

bool Joystick::Right()
{
    return readX() > (_centerX + _deadZoneX);
}

bool Joystick::Left()
{
    return readX() < (_centerX - _deadZoneX);
}
bool Joystick::zero()
{
    int x = readX();
    int y = readY();

    int dx = x - _centerX;
    if (dx < 0)
        dx = -dx; // abs

    int dy = y - _centerY;
    if (dy < 0)
        dy = -dy;

    return (dx <= _deadZoneX) && (dy <= _deadZoneY);
}

bool Joystick::result()
{
    if (readX() > readY())
    {
        if (Right())
            printf("Right\n");
        else if (Up())
            printf("Up\n");
    }
    else if (readY() > readX())
    {
        if (Down())
            printf("Down\n");
        else if (Left())
            printf("Left\n");
    }
    else if (zero())
    {
        printf("zero\n");
    }
    return true;
}

bool Joystick::Pressed()
{
    // Lectura raw (con pull-up: not pressed => HIGH (1), pressed => LOW (0))
    bool raw_level = _button.get() != 0; // true when HIGH (not pressed)
    bool pressed_raw = !raw_level;       // true when physically pressed

    uint64_t now = esp_timer_get_time();

    if (pressed_raw != _btn_last_state)
    {
        // hubo cambio en lectura raw, reiniciamos timer de estabilidad
        _btn_last_state = pressed_raw;
        _btn_last_change_us = now;
        return false; // esperar a estabilidad
    }

    // si ha estado estable el tiempo de debounce y el estado estable cambió
    if ((now - _btn_last_change_us) >= _btn_debounce_us)
    {
        if (_btn_stable_state != pressed_raw)
        {
            // estado estable cambió -> si es press (rising logical for pressed_raw true) devolvemos true una vez
            _btn_stable_state = pressed_raw;
            if (pressed_raw)
            {
                return true; // evento one-shot de pulsación
            }
            // si es liberación (pressed_raw == false) devolvemos false (no queremos evento)
        }
    }

    return false;
}