#include "Ultrasonic.h"
#include <esp_log.h>

// Wrapper IRAM-safe para la ISR
static void IRAM_ATTR ultrasonic_isr_wrapper(void *arg)
{
    Ultrasonic *self = static_cast<Ultrasonic *>(arg);
    if (self) self->handler();
}

Ultrasonic::Ultrasonic() { }

Ultrasonic::~Ultrasonic()
{
    // Remover solo nuestro handler (no desinstales el servicio global)
    if (_gpio_echo != GPIO_NUM_NC) {
        gpio_isr_handler_remove(_gpio_echo);
    }
}

void Ultrasonic::setup(uint8_t gpio_echo, uint8_t gpio_trig, uint8_t ch_trig, TimerConfig timer_config)
{
    // Configura trigger PWM (igual que antes)
    trigger.setup(gpio_trig, ch_trig, &timer_config, 0);
    trigger.setDuty(0.1f);

    // Configura pin echo
    _gpio_echo = (gpio_num_t)gpio_echo;
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    // activamos pull-up o pull-down según tu circuito; si el pin queda flotante usa pull-up
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pin_bit_mask = (1ULL << _gpio_echo);
    gpio_config(&io_conf);

    // NO desinstales/instales el servicio ISR aquí (hace interferencia con otros módulos)
    // Asume que app_main ya llamó gpio_install_isr_service(ESP_INTR_FLAG_IRAM).

    // Añade handler mínimo e IRAM-safe
    gpio_isr_handler_add(_gpio_echo, ultrasonic_isr_wrapper, this);
}

float Ultrasonic::getDistance()
{
    // leer variable volatile de forma segura (copia local)
    uint64_t echo_time_local = _echo_time;
    // velocidad del sonido 343 m/s -> 0.343 mm/us. /2 por ida+vuelta
    return 0.343f * (float)echo_time_local / 2.0f; // mm
}

void IRAM_ATTR Ultrasonic::handler()
{
    // Mínimo trabajo en ISR: leer nivel y timestamp rápido
    uint8_t level = gpio_get_level(_gpio_echo);
    uint64_t now = esp_timer_get_time(); // esta función suele ser rápida; si te preocupa, usar contador HW

    // filtro anti-rebote: ignorar flancos muy cercanos (< 100 us)
    uint64_t prev = _prev_micros;
    if (now - prev < 100) {
        // si el delta es demasiado pequeño, ignoramos (evita ruidos)
        _states[1] = _states[0];
        _states[0] = level;
        return;
    }

    _states[0] = level;
    if (_states[0] > _states[1]) {
        // borde de subida -> registrar tiempo
        _prev_micros = now;
    } else {
        // borde de bajada -> calcular ancho de pulso
        _echo_time = now - _prev_micros;
    }
    _states[1] = _states[0];
}
