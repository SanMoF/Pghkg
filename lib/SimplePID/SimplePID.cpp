#include "SimplePID.h"
#include <algorithm> // std::clamp
#include <cmath>

void SimplePID::setup(float Kp, float Ki, float Kd, float dt, float outMin, float outMax)
{
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;

    // proteger dt > 0
    _dt = (dt > 0.0f) ? dt : 0.01f;

    _outMin = outMin;
    _outMax = outMax;

    // reset state
    _I = 0.0f;
    _prevError = 0.0f;
    _firstCycle = true;
}


float SimplePID::apply(float setpoint, float measurement)
{
    
    float error = setpoint - measurement;

    // Proportional
    float P = _Kp * error;

    // Derivative (difference quotient). Skip on first cycle.
    float D = 0.0f;
    if (!_firstCycle)
    {
        D = _Kd * (error - _prevError) / _dt;
    }
    else
    {
        _firstCycle = false;
    }

    // Anti-windup: predict integrator and only accept it if it doesn't
    // keep the output saturated (or if it helps desaturar).
    float I_pred = _I + error * _dt;      // integrator predicted (accumulated error)
    float I_term_pred = _Ki * I_pred;     // predicted integral contribution

    // unsaturated output using predicted integrator
    float u_unsat = P + I_term_pred + D;

    // saturate
    float u = std::clamp(u_unsat, _outMin, _outMax);

    // decide whether to accept integration step
    bool saturatedHigh = (u_unsat > _outMax);
    bool saturatedLow  = (u_unsat < _outMin);

    bool integrate = false;
    if (!saturatedHigh && !saturatedLow)
    {
        integrate = true; // not saturated => integrate normally
    }
    else if (saturatedHigh && (error < 0.0f))
    {
        // above max but error negative would reduce output -> allow integrate
        integrate = true;
    }
    else if (saturatedLow && (error > 0.0f))
    {
        // below min but error positive would increase output -> allow integrate
        integrate = true;
    }

    if (integrate)
    {
        _I = I_pred;

        // clamp integrator to reasonable bounds to avoid numeric explosion
        if (_Ki != 0.0f)
        {
            float maxI = _outMax / _Ki;
            float minI = _outMin / _Ki;
            _I = std::clamp(_I, minI, maxI);
        }
    }
    // else: keep old _I (no integration)
    float Integ = _Ki*_I;

    // save for next cycle
    _prevError = error;

    // recompute final output with accepted integrator
    float output = _Kp * error + Integ + D;

    return output;
}
