#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__
// Ext Libs
#include <stdio.h>
#include <string.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// My Libs
#include "SimpleGPIO.h"
#include "SimpleTimer.h"
#include "WifiManager.h"
#include "MqttManager.h"
#include "ServoStepper.h"
#include "Robotics.h"
#include "Stepper.h"

// ─── Timing ───────────────────────────────────────────────────────
float dt = 10000;

// ─── Peripheral objects ───────────────────────────────────────────
SimpleTimer Timer;
WifiManager wifi;
MqttManager mqtt;
ServoStepper Base_Stepper;
Stepper Z_Stepper; // ← new

// Pin Definitions
// Base Stepper
uint8_t Base_Dir_Pin = 5;
uint8_t Base_PWM_Pin = 4;
uint8_t Base_PWM_Ch = 0;

// Z axis Stepper

uint8_t Z_Stepper_Dir_Pin = 19;
uint8_t Z_Stepper_PWM_Pin = 18;
uint8_t Z_Stepper_PWM_Ch = 1;

// Stepper 1 — independent frequency control
static TimerConfig STEPPER_TIMER_0{
    .timer = LEDC_TIMER_0,
    .frequency = 5000,
    .bit_resolution = LEDC_TIMER_12_BIT,
    .mode = LEDC_LOW_SPEED_MODE};

// Stepper 2 — independent frequency control
static TimerConfig STEPPER_TIMER_1{
    .timer = LEDC_TIMER_1,
    .frequency = 4050,
    .bit_resolution = LEDC_TIMER_12_BIT,
    .mode = LEDC_LOW_SPEED_MODE};

// DC motors — both share this, duty is per-channel
static TimerConfig DC_TIMER{
    .timer = LEDC_TIMER_2,
    .frequency = 20000,
    .bit_resolution = LEDC_TIMER_8_BIT,
    .mode = LEDC_HIGH_SPEED_MODE};
// ─── Network ──────────────────────────────────────────────────────
#define WIFI_SSID "WIFI_RAY"
#define WIFI_PASSWORD "Santi2011"
#define MQTT_BROKER_URI "mqtt://192.168.80.158:1883"
#define MQTT_CLIENT_ID "ESP32_Client_01"
#define TOPIC_PUB "esp32/status"
#define TOPIC_SUB "esp32/commands"

// ─── MQTT command globals ─────────────────────────────────────────
int mode = -1;
float theta1 = 0, theta2 = 0, theta3 = 0, theta4 = 0;
float cmd_x = 0, cmd_y = 0, cmd_z = 0;
//PID Gains
float base_gains[3] = {8, 1.48, 0.0};
float z_gains[3] = {10.0f, 0.0f, 0.0f}; // tune for Z axis
// In definitions.h, add this global:
char pub_buf[32];
#endif // __DEFINITIONS_H__