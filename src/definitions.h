#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__
#include "ServoStepper.h"
#include "SimpleTimer.h"

SimpleTimer timer;

float dt = 10000;


static TimerConfig PWM_STEPPER_TIMER{
    .timer = LEDC_TIMER_0,
    .frequency = 650,
    .bit_resolution = LEDC_TIMER_8_BIT,
    .mode = LEDC_LOW_SPEED_MODE};

#endif // __DEFINITIONS_H__