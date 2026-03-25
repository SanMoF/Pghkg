#include "LineF.h"

LineF::LineF()
{
}

void LineF::setup(const uint8_t LF_pin[2])
{
    LF[0].setup(LF_pin[0]);
    LF[1].setup(LF_pin[1]);
}

float LineF::Read()
{
    return LF[0].read(ADC_READ_RAW) - LF[1].read(ADC_READ_RAW);
}
