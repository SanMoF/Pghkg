#include "DCMotor.h"
#include <stdio.h>

DCMotor::DCMotor()
    : _mode(DCMotorMode::VELOCITY),
      _targetVelocity(0.0f),
      _targetPosition(0.0f),
      _currentVelocity(0.0f),
      _currentPosition(0.0f)
{}

void DCMotor::setup(uint8_t motorPins[2],
                    uint8_t motorChannels[2],
                    uint8_t encoderPins[2],
                    TimerConfig* timerCfg,
                    float* velocityGains,
                    float* positionGains,
                    uint64_t dt)
{
    _motor.setup(motorPins, motorChannels, timerCfg);
    _encoder.setup(encoderPins, 0.36445f);      // deg/tick — adjust to your encoder

    _velPid.setup(velocityGains, (float)dt);
    _velPid.setULimit(100.0f);

    _posPid.setup(positionGains, (float)dt);
    _posPid.setULimit(100.0f);

    _motor.setSpeed(0.0f);
}

// ─── Public API ───────────────────────────────────────────────────

void DCMotor::setMode(DCMotorMode mode)
{
    if (_mode == mode) return;
    _mode = mode;
    _velPid.reset();
    _posPid.reset();
    _motor.setSpeed(0.0f);
}

void DCMotor::setTargetVelocity(float rpm)
{
    _targetVelocity = rpm;
    if (rpm == 0.0f) {
        _motor.setSpeed(0.0f);
        _velPid.reset();
    }
}

void DCMotor::setTargetPosition(float degrees)
{
    if (degrees == _targetPosition) return;
    _targetPosition = degrees;
    _posPid.reset();
}

float DCMotor::getVelocity()  { return _currentVelocity; }
float DCMotor::getPosition()  { return _currentPosition; }

// ─── update() — call every timer tick ────────────────────────────

void DCMotor::update()
{
    _currentVelocity = _encoder.getSpeed();
    _currentPosition = _encoder.getAngle();

    switch (_mode) {
        case DCMotorMode::VELOCITY: _applyVelocityControl(); break;
        case DCMotorMode::POSITION: _applyPositionControl(); break;
    }
}

// ─── Private control loops ────────────────────────────────────────

void DCMotor::_applyVelocityControl()
{
    float error = _targetVelocity - _currentVelocity;
    float u = _velPid.computedU(error);

    if (u >  100.0f) u =  100.0f;
    if (u < -100.0f) u = -100.0f;

    _motor.setSpeed(u);
}

void DCMotor::_applyPositionControl()
{
    float error = _targetPosition - _currentPosition;
    float u = _posPid.computedU(error);

    if (u >  100.0f) u =  100.0f;
    if (u < -100.0f) u = -100.0f;

    _motor.setSpeed(u);
}