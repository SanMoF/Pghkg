# Conexiones — Balancín BLDC (ESP32)

Pines definidos en `src/definitions.h`. Si cambiás alguno ahí, actualizá este archivo.

## MPU6050 (IMU, sensor de balance)

| MPU6050 (GY-521) | ESP32          |
|-------------------|----------------|
| VCC                | 3.3V           |
| GND                | GND            |
| SDA                | GPIO 21        |
| SCL                | GPIO 22        |

- Bus: `I2C_NUM_0`, 100 kHz (`lib/MPU6050/include/MPU6050.h`). No usar 400 kHz en breadboard — el driver está deliberadamente en 100 kHz porque a 400 kHz un jumper ruidoso tiende a mostrar silencio total (todo NACK) en vez de un error claro.
- La mayoría de las placas GY-521 ya traen pull-ups de I2C a bordo. Si usás el chip MPU6050 pelado (sin breakout), agregá pull-ups externas de 4.7 kΩ de SDA y SCL a 3.3V.
- Dirección I2C esperada: `0x68` (o `0x69` si el pin `AD0` está en alto).
- **El robot debe estar quieto e inmóvil durante el boot** — `calibrateGyro()` se ejecuta una sola vez en `app_main()` y promedia el offset de giro asumiendo reposo.

## ESCs BLDC (motores de las ruedas)

Señal PWM tipo servo a 50 Hz, 1000–2000 µs. Sin reversa (ESCs unidireccionales) — ver comentario en `lib/BLDC_ESC/include/BLDC_ESC.h`.

| Motor  | Señal ESC → ESP32 | Canal LEDC |
|--------|--------------------|------------|
| ESC 1  | GPIO 18            | 0          |
| ESC 2  | GPIO 19            | 1          |

- Compartir **GND común** entre ESP32 y los ESCs (referencia de la señal PWM).
- **No** alimentar el ESP32 desde el BEC del ESC ni al revés salvo que sepas que las tierras y niveles son compatibles — la alimentación de potencia de los motores va directo de la batería a los ESCs, separada del ESP32.
- Ambos ESCs comparten un mismo timer LEDC a 50 Hz (`ESC_TIMER` en `src/definitions.h`), un canal cada uno.
- Al encender: los ESCs deben recibir pulso de idle (mínimo) antes de que llegue cualquier throttle, o algunos se niegan a armar y solo pitan error. `app_main()` ya hace `arm()` + `vTaskDelay(2500 ms)` antes de aceptar comandos — no lo saltees.

## Consola UART

- UART0 (USB), 115200 baudios — el mismo puerto USB que usás para flashear (`pio device monitor`). No requiere cableado extra.

## Orden recomendado al conectar

1. ESP32 sin batería de motores conectada todavía.
2. MPU6050: VCC, GND, SDA (21), SCL (22).
3. Señal de ESC1 (18) y ESC2 (19), y GND común ESP32↔ESCs.
4. Recién al final, batería de los ESCs/motores — con el ESP32 ya energizado y el firmware corriendo (para que reciban el pulso de idle correcto al armar).
