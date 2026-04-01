#include "ServoStepper.h"
#include <math.h>

// ─────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────
void ServoStepper::setup(uint8_t dir_pin, uint8_t step_pin,
                         uint8_t pwm_channel, TimerConfig *timer,
                         float pid_gains[3], float pid_limit)
{
    _dir.setup(dir_pin, GPO);
    _step.setup(step_pin, pwm_channel, timer);
    _step.setDuty(0.0f);

    _pid.setup(pid_gains, pid_limit);
    _pid.setULimit(6000);

    _encoder.begin();

    uint8_t status = 0;
    do {
        _encoder.getStatus(&status);
    } while (!(status & AS5600_STATUS_MD));
}

// ─────────────────────────────────────────────────────────────
//  Shared drive logic
// ─────────────────────────────────────────────────────────────
void ServoStepper::_applyControl(float error)
{
    if (fabsf(error) < SERVO_DEADBAND_DEG) {
        _step.setDuty(0.0f);
        return;
    }

    float u    = _pid.computedU(error);
    float freq = fabsf(u);

    if (freq < 100.0f) {          // below LEDC minimum → stop cleanly
        _step.setDuty(0.0f);
        return;
    }

    _dir.set(u > 0 ? 0 : 1);
    _step.setFrequency(freq);
    _step.setDuty(50.0f);
}

// ─────────────────────────────────────────────────────────────
//  Absolute multi-turn
// ─────────────────────────────────────────────────────────────
void ServoStepper::goToAngle(float target_deg)
{
    float angle = _encoder.getAccumulatedAngleDeg();
    if (isnan(angle)) { _step.setDuty(0.0f); return; }

    _applyControl(target_deg - angle);
}

// ─────────────────────────────────────────────────────────────
//  Relative 0–360, shortest path
// ─────────────────────────────────────────────────────────────
void ServoStepper::moveToRelAngle(float target_deg)
{
    // Clamp target to [0, 360)
    target_deg = fmodf(target_deg, 360.0f);
    if (target_deg < 0.0f) target_deg += 360.0f;

    float current = _encoder.getAngleDeg();   // always 0–360, no unwrap
    if (isnan(current)) { _step.setDuty(0.0f); return; }

    // Shortest angular path: wrap delta into (-180, +180]
    float error = target_deg - current;
    if (error >  180.0f) error -= 360.0f;
    if (error <= -180.0f) error += 360.0f;

    _applyControl(error);
}