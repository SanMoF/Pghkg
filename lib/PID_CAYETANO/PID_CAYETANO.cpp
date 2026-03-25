// PID_CAYETANO.cpp
#include "PID_CAYETANO.h"
#include <stdio.h>

// Uncomment to enable runtime debug prints
#define PID_DEBUG 0

PID_CAYETANO::PID_CAYETANO()
{
    _Kp = _Ki = _Kd = 0.0f;
    _dt = 0.0f;
    _u_limit = 1000.0f; // Default to 1000 Hz max frequency
    integral = 0.0f;
    prev_error = 0.0f;
}

void PID_CAYETANO::setup(float Gains[3], float dt_us)
{
    _Kp = Gains[0];
    _Ki = Gains[1];
    _Kd = Gains[2];
    
    // dt_us is already in microseconds, convert to seconds
    _dt = dt_us / 1000000.0f;
    
    integral = 0.0f;
    prev_error = 0.0f;
    
    // Debug: verify dt was set correctly
    printf("PID setup: dt=%.6f seconds (from %.0f us)\n", _dt, dt_us);
    printf("PID gains: Kp=%.4f Ki=%.4f Kd=%.4f\n", _Kp, _Ki, _Kd);
}

void PID_CAYETANO::setULimit(float ul)
{
    if (ul <= 0.0f) {
        printf("PID WARNING: Invalid U limit %.2f, keeping %.2f\n", ul, _u_limit);
        return;
    }
    _u_limit = ul;
    printf("PID: U limit set to %.2f Hz\n", _u_limit);
}

float PID_CAYETANO::computedU(float error)
{
    // Safety check: dt must be positive
    if (_dt <= 0.0f) {
        printf("PID ERROR: _dt = %.9f (should be > 0)\n", _dt);
        return 0.0f;
    }

    // Proportional term
    float P = _Kp * error;

    // Derivative term (backward difference)
    float D = _Kd * (error - prev_error) / _dt;

    // Trapezoidal integration (more accurate than Euler)
    float error_avg = 0.5f * (error + prev_error);
    float integral_update = error_avg * _dt;
    float tentative_integral = integral + integral_update;

    // Tentative output if we accept the integral update
    float u_tentative = P + _Ki * tentative_integral + D;

    // Anti-windup: only integrate if output is within limits
    if (u_tentative > -_u_limit && u_tentative < _u_limit) {
        integral = tentative_integral;
    }
    // else: don't update integral (prevents windup)

    // Clamp integral to prevent runaway
    float integral_limit = _u_limit * 2.0f;
    if (integral > integral_limit) integral = integral_limit;
    if (integral < -integral_limit) integral = -integral_limit;

    // Final output
    float u = P + _Ki * integral + D;

    // Clamp output to configured limits
    if (u > _u_limit) u = _u_limit;
    if (u < -_u_limit) u = -_u_limit;

    // Save error for next iteration
    prev_error = error;

#if PID_DEBUG
    printf("PID: e=%.2f | P=%.2f I=%.2f D=%.2f | u=%.2f (lim=%.0f)\n",
           error, P, _Ki * integral, D, u, _u_limit);
#endif

    return u;
}

void PID_CAYETANO::reset()
{
    integral = 0.0f;
    prev_error = 0.0f;
    printf("PID: Reset\n");
}