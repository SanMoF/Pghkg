#include "Keyboard.h"

Keyboard::Keyboard()
{
}

void Keyboard::setup(const uint8_t pin[8])
{
    for (int i = 0; i < 8; i++)
        key[i].setup(pin[i], GPI, GPIO_PULLUP_ONLY);
}

int Keyboard::get()
{
    int row, col;
    for (int i = 0; i < 4; i++)
    {
        if (key[i].get())
            row = i;
        if (key[i + 4].get())
            col = i;
    }
    return matrix[row][col];
}
