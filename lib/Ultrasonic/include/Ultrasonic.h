#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "SimplePWM.h"
#include "driver/gpio.h"
#include "esp_timer.h"

class Ultrasonic
{
private:
    SimplePWM trigger;
    gpio_num_t _gpio_echo = GPIO_NUM_NC;

    // variables actualizadas en ISR => volatile
    volatile uint64_t _prev_micros = 0;
    volatile uint64_t _echo_time = 0;
    volatile uint8_t _states[2] = {0, 0};

public:
    Ultrasonic();
    ~Ultrasonic();
    
    void setup(uint8_t gpio_echo, uint8_t gpio_trig, uint8_t ch_trig, TimerConfig timer_config);
    
    float getDistance(); // mm
    void handler();      // ejecutado desde ISR (llamado por wrapper IRAM_ATTR)
};

#endif // ULTRASONIC_H
