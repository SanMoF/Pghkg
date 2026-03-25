#ifndef __LINEF_H__
#define __LINEF_H__

#include "SimpleADC.h"

class LineF
{
    public:
    LineF();
    void setup(const uint8_t LF_pin[2]);
    float Read();
    
    private:
    SimpleADC LF[2];
};

#endif // __LINEF_H__