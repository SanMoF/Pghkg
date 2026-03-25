#ifndef __SIMPLEPID_H__
#define __SIMPLEPID_H__



class SimplePID
{
public:
    SimplePID() = default;
    ~SimplePID() = default;

    // Configura Kp, Ki, Kd, periodo dt (segundos) y límites de salida [outMin, outMax].
    // dt debe ser > 0 (se valida en setup).
    void setup(float Kp, float Ki, float Kd, float dt = 0.01f,
               float outMin = 100.0f, float outMax = 100.0f);

    // Calcula y devuelve la salida del PID usando setpoint y measurement.
    // apply() llama internamente a get_error().
    float apply(float setpoint, float measurement);

    // Devuelve el error (setpoint - measurement) — implementado como método público.

private:
    // parámetros
    float _Kp = 0.0f;
    float _Ki = 0.0f;
    float _Kd = 0.0f;
    float _dt = 0.01f;

    // estado interno
    float _I = 0.0f;         // integrador (acumulador, sin multiplicar por Ki)
    float _prevError = 0.0f; // error anterior (para derivada)
    bool  _firstCycle = true;

    // límites de salida
    float _outMin = -1e6f;
    float _outMax =  1e6f;
};

#endif // __SIMPLEPID_H__
