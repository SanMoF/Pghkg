#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

// ─── External Libraries ───────────────────────────────────────────
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

// ─── Project Libraries ────────────────────────────────────────────
#include "SimpleGPIO.h"
#include "SimpleTimer.h"
#include "WifiManager.h"
#include "MqttManager.h"
#include "ServoStepper.h"
#include "Stepper.h"
#include "Hbridge.h"
#include "QuadratureEncoder.h"
#include "Robotics.h"
#include "DCMotor.h"
#include "Robotics.h"

// ─── Timing ───────────────────────────────────────────────────────
float dt = 10000; // us — 10 ms control loop

// ─── Pin Definitions ──────────────────────────────────────────────

// Base stepper (ServoStepper — closed loop)
uint8_t BASE_DIR_PIN = 5;
uint8_t BASE_PWM_PIN = 4;
uint8_t BASE_PWM_CH = 0;

// Z axis stepper (open loop)
uint8_t Z_DIR_PIN = 19;
uint8_t Z_PWM_PIN = 18;
uint8_t Z_PWM_CH = 1;

// DC motor arm
uint8_t DC_PINS[2] = {13,12};
uint8_t DC_CH [2] = {2,3};
uint8_t ENC_PINS[2] = {25, 26};


// ─── LEDC Timer Configs ───────────────────────────────────────────

static TimerConfig STEPPER_TIMER_0{// Base stepper
                                   .timer = LEDC_TIMER_0,
                                   .frequency = 5000,
                                   .bit_resolution = LEDC_TIMER_12_BIT,
                                   .mode = LEDC_LOW_SPEED_MODE};

static TimerConfig STEPPER_TIMER_1{// Z stepper
                                   .timer = LEDC_TIMER_1,
                                   .frequency = 4050,
                                   .bit_resolution = LEDC_TIMER_12_BIT,
                                   .mode = LEDC_LOW_SPEED_MODE};

static TimerConfig DC_TIMER{// DC motors (shared)
                            .timer = LEDC_TIMER_2,
                            .frequency = 20000,
                            .bit_resolution = LEDC_TIMER_8_BIT,
                            .mode = LEDC_HIGH_SPEED_MODE};

// ─── Peripheral Objects ───────────────────────────────────────────
SimpleTimer Timer;
WifiManager wifi;
MqttManager mqtt;
ServoStepper Base_Stepper;
Stepper Z_Stepper;
HBridge DC_Motor;
QuadratureEncoder Encoder_arm;
DCMotor arm_motor;

// ─── PID Gains ────────────────────────────────────────────────────
float base_gains[3] = {8.0f, 1.48f, 0.0f};
float z_gains[3] = {10.0f, 0.0f, 0.0f};
float arm_pos_gains[3] = { 3.0f, 0.1f, 0.1f  };   // tune
float arm_vel_gains[3] = { 2.0f, 0.5f, 0.05f };


// ─── MQTT Command Globals ─────────────────────────────────────────
int mode = -1;
float theta1 = 0, theta2 = 0, theta3 = 0, theta4 = 0;
float cmd_x = 0, cmd_y = 0, cmd_z = 0;

// ─── Network ──────────────────────────────────────────────────────
#define WIFI_SSID "WIFI_RAY"
#define WIFI_PASSWORD "Santi2011"
#define MQTT_BROKER_URI "mqtt://192.168.80.158:1883"
#define MQTT_CLIENT_ID "ESP32_Client_01"
#define TOPIC_PUB "esp32/status"
#define TOPIC_SUB "esp32/commands"

// ─── Misc ─────────────────────────────────────────────────────────
char pub_buf[32];

#endif // __DEFINITIONS_H__