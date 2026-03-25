#include "Buzzer.h"

Buzzer::Buzzer() {
    _pin = 0;
    _channel = 0;
    _timer = nullptr;
    _melody = nullptr;
    _durations = nullptr;
    _length = 0;
    _index = 0;
    _playing = false;
    _currentFreq = 0;
    _currentDuration = 0;
    _noteStartTime = 0;
}

Buzzer::~Buzzer() {}

void Buzzer::setup(uint8_t pin, uint8_t channel, TimerConfig* timer) {
    _pin = pin;
    _channel = channel;
    _timer = timer;
    pwm.setup(_pin, _channel, _timer);
    pwm.setDuty(50);
}

void Buzzer::playTone(int frequency, int duration_ms) {
    _currentFreq = frequency;
    _currentDuration = duration_ms;
    _noteStartTime = esp_timer_get_time() / 1000; // en ms
    _playing = true;

    if (frequency > 0) {
        pwm.setFrequency(frequency);
        pwm.setDuty(50);
    } else {
        pwm.setDuty(0);
    }
}

void Buzzer::playMelody(const int* melody, const int* durations, int length) {
    _melody = melody;
    _durations = durations;
    _length = length;
    _index = 0;
    playTone(_melody[_index], _durations[_index]);
}

void Buzzer::update() {
    if (!_playing) return;

    int64_t now = esp_timer_get_time() / 1000; // ms
    if (now - _noteStartTime >= _currentDuration) {
        stop();
        _playing = false;

        // Pasar a la siguiente nota si hay melodía
        if (_melody && _index + 1 < _length) {
            _index++;
            playTone(_melody[_index], _durations[_index]);
        }
    }
}

void Buzzer::stop() {
    pwm.setDuty(0);
}

// 🚀 Nueva función
void Buzzer::setFrequency(int frequency) {
    if (frequency > 0) {
        _currentFreq = frequency;
        pwm.setFrequency(frequency);
        pwm.setDuty(50); // mantener duty al 50% para sonido estable
    } else {
        pwm.setDuty(0); // silencio si frecuencia <= 0
    }
}
