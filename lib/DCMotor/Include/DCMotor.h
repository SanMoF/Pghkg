#pragma once

#include <stdint.h>
#include "Hbridge.h"
#include "QuadratureEncoder.h"
#include "PID_CAYETANO.h"
#include "driver/ledc.h"

enum class DCMotorMode {
    VELOCITY,
    POSITION
};

class DCMotor {
public:
    DCMotor();

    void setup(uint8_t motorPins[2],
               uint8_t motorChannels[2],
               uint8_t encoderPins[2],
               TimerConfig* timerCfg,
               float* velocityGains,
               float* positionGains,
               uint64_t dt);

    void update();

    void setMode(DCMotorMode mode);
    void setTargetVelocity(float rpm);
    void setTargetPosition(float degrees);

    float getVelocity();
    float getPosition();

private:
    HBridge             _motor;
    QuadratureEncoder   _encoder;
    PID_CAYETANO        _velPid;
    PID_CAYETANO        _posPid;

    DCMotorMode _mode;

    float _targetVelocity;
    float _targetPosition;
    float _currentVelocity;
    float _currentPosition;

    void _applyVelocityControl();
    void _applyPositionControl();
};