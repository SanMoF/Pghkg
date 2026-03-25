// Viscometer.h
#ifndef VISCOMETER_H
#define VISCOMETER_H

#include "Hbridge.h"
#include "QuadratureEncoder.h"
#include "PID_CAYETANO.h"
#include "SimpleADC.h"
struct ViscometerReading {
    float viscosity; // Cps
    float rpm;
};

class Viscometer
{
public:
    HBridge motor;
    QuadratureEncoder encoder;
    PID_CAYETANO pid;
    SimpleADC ADC;

    float targetSpeed;
    float currentSpeed;

    Viscometer();
    ~Viscometer();

    // antes: void setup(..., TimerConfig timerCfg, uint64_t dt);
    // prototype
    void setup(uint8_t motorPins[2],
               uint8_t motorChannels[2],
               uint8_t encoderPins[2],
               TimerConfig *timerCfg, uint64_t dt, uint8_t ADC_PIN);

    ViscometerReading  measure();
    void setTargetSpeed(float speed);
};


#endif