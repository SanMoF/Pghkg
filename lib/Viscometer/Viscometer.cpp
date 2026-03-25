#include "Viscometer.h"
#include <stdio.h>

Viscometer::Viscometer()
{
    targetSpeed = 0.0f;
    currentSpeed = 0.0f;
}

Viscometer::~Viscometer()
{
    motor.setSpeed(0.0f);
}

void Viscometer::setup(uint8_t motorPins[2],
                       uint8_t motorChannels[2],
                       uint8_t encoderPins[2],
                       TimerConfig *timerCfg, uint64_t dt, uint8_t ADC_PIN)
{
    motor.setup(motorPins, motorChannels, timerCfg);
    encoder.setup(encoderPins, 0.36445f);
    ADC.setup(ADC_PIN);
    
    // FIXED: Stronger PID gains for better speed control
    // Kp = 2.0 (proportional gain - responds to current error)
    // Ki = 0.5 (integral gain - eliminates steady-state error)
    // Kd = 0.05 (derivative gain - dampens oscillations)
    float gains[3] = {0.5f, 1.0f, 0.0f};
    pid.setup(gains, (float)dt);
    pid.setULimit(100.0f); // Limit output to ±100%
    
    motor.setSpeed(0.0f);
}

ViscometerReading Viscometer::measure()
{
    ViscometerReading m;
    
    // Read current speed from encoder
    currentSpeed = encoder.getSpeed();
    
    // Calculate error between target and current speed
    float error = targetSpeed - currentSpeed;
    
    // Compute PID control output
    float u = pid.computedU(error);
    
    // Clamp output to valid motor speed range [-100, 100]
    if (u > 100.0f) u = 100.0f;
    if (u < -100.0f) u = -100.0f;
    
    // Apply control signal to motor
    motor.setSpeed(u);
    
    // Convert encoder speed to RPM
    // Original conversion: currentSpeed * 0.17
    // If this doesn't match your system, adjust the factor
    m.rpm = currentSpeed * 0.17f;
    
    // Read viscosity from load sensor (ADC)
    m.viscosity = 1.7607*ADC.read(ADC_READ_RAW)-1402;
    
    // Debug output every ~100 calls (adjust as needed)
    static int debug_counter = 0;
    if (++debug_counter >= 100)
    {
        printf("Viscometer: target=%.1f current=%.1f error=%.1f u=%.1f rpm=%.1f\n",
               targetSpeed, currentSpeed, error, u, m.rpm);
        debug_counter = 0;
    }
    
    return m;
}

void Viscometer::setTargetSpeed(float speed)
{
    // FIXED: Set the PID target, not the motor directly
    targetSpeed = speed;
    
    // If stopping (speed == 0), also immediately set motor to 0
    if (speed == 0.0f)
    {
        motor.setSpeed(0.0f);
        pid.reset(); // Reset PID integrator to prevent windup
    }
    
    printf("Viscometer: target speed set to %.1f (%.1f RPM)\n", speed, speed * 0.17f);

}