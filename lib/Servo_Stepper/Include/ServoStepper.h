#pragma once

#include "AS5600.h"
#include "SimpleGPIO.h"
#include "SimplePWM.h"
#include "PID_CAYETANO.h"

#define SERVO_DEADBAND_DEG 0.5f

class ServoStepper
{
public:
AS5600       _encoder;
    void setup(uint8_t dir_pin, uint8_t step_pin,
               uint8_t pwm_channel, TimerConfig *timer,
               float pid_gains[3], float pid_limit);

    void goToAngle(float target_deg);       // absolute, multi-turn
    void moveToRelAngle(float target_deg);  // 0–360 only, shortest path

private:
    SimpleGPIO   _dir;
    SimplePWM    _step;
    PID_CAYETANO _pid;

    void _applyControl(float error);        // shared drive logic
};