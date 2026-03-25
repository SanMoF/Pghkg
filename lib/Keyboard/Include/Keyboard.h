#ifndef __KEYBOARD_H__
#define __KEYBOARD_H__

#include "SimpleGPIO.h"

class Keyboard
{
public:
    Keyboard();
    void setup(const uint8_t pin[8]);
    int get();

private:
    SimpleGPIO key[8];
    int line[8];
    const char matrix[4][4] = {{'1', '2', '3', 'A'}, 
                               {'4', '5', '6', 'B'}, 
                               {'7', '8', '9', 'C'}, 
                               {'*', '0', '#', 'D'}};
};
#endif // __KEYBOARD_H__