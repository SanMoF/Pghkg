#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "SimpleTimer.h"
#include "WifiManager.h"
#include "MqttManager.h"
#include "ServoStepper.h"
#include "nvs_flash.h"


// ─── Timing ───────────────────────────────────────────────────────
float dt = 10000;
SimpleTimer Timer;

// ─── Peripheral objects ───────────────────────────────────────────
WifiManager wifi;
MqttManager mqtt;
ServoStepper motor;

// ─── PWM config ───────────────────────────────────────────────────
static TimerConfig PWM_STEPPER_TIMER {
    .timer          = LEDC_TIMER_0,
    .frequency      = 650,
    .bit_resolution = LEDC_TIMER_8_BIT,
    .mode           = LEDC_LOW_SPEED_MODE
};

// ─── Network ──────────────────────────────────────────────────────
#define WIFI_SSID       "WIFI_RAY"
#define WIFI_PASSWORD   "Santi2011"
#define MQTT_BROKER_URI "mqtt://192.168.80.158:1883"
#define MQTT_CLIENT_ID  "ESP32_Client_01"
#define TOPIC_PUB       "esp32/status"
#define TOPIC_SUB       "esp32/commands"

// ─── MQTT command globals ─────────────────────────────────────────
int   mode   = -1;
float theta1 = 0, theta2 = 0, theta3 = 0, theta4 = 0;
float cmd_x  = 0, cmd_y  = 0, cmd_z  = 0;

#endif // __DEFINITIONS_H__