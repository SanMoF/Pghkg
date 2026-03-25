// PID_CAYETANO.h
#ifndef __PID_CAYETANO_H__
#define __PID_CAYETANO_H__

class PID_CAYETANO
{
private:
    float _Kp;
    float _Ki;
    float _Kd;
    float _dt;
    float _u_limit;      // NEW: configurable output limit
    float prev_error;
    float integral;

public:
    PID_CAYETANO();
    void setup(float Gains[3], float dt_us);
    void setULimit(float ul);  // NEW: set output limit (Hz)
    float computedU(float error);
    void reset();
};

#endif // __PID_CAYETANO_H__