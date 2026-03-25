#ifndef _BUZZER_H_
#define _BUZZER_H_

#include "SimplePWM.h"
#include "esp_timer.h"

class Buzzer {
public:
    Buzzer();
    ~Buzzer();

    void setup(uint8_t pin, uint8_t channel, TimerConfig* timer);
    void playTone(int frequency, int duration_ms);
    void playMelody(const int* melody, const int* durations, int length);
    void update();
    void stop();

    // 🚀 Nueva función para cambiar frecuencia directamente
    void setFrequency(int frequency);

private:
    SimplePWM pwm;
    uint8_t _pin;
    uint8_t _channel;
    TimerConfig* _timer;

    // Estado interno
    const int* _melody;
    const int* _durations;
    int _length;
    int _index;
    int _currentFreq;
    int _currentDuration;
    int64_t _noteStartTime;
    bool _playing;
};

/// 🎶 Notas musicales (frecuencias en Hz)
#define NOTE_C4   262
#define NOTE_D4   294
#define NOTE_E4   330
#define NOTE_F4   349
#define NOTE_FS4  370
#define NOTE_G4   392
#define NOTE_A4   440
#define NOTE_B4   494
#define NOTE_C5   523
#define NOTE_D5   587
#define NOTE_AS4  466
#define NOTE_CS5  554
#define NOTE_DS5  622
#define NOTE_E5   659
#define NOTE_F5   698
#define NOTE_FS5  740
#define NOTE_G5   784
#define NOTE_GS5  830
#define NOTE_A5   880
#define NOTE_AS5  932
#define NOTE_B5   988
#define NOTE_C6   1047

#endif // _BUZZER_H_
