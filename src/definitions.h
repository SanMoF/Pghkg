#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

// ─── External Libraries ───────────────────────────────────────────
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// ─── Project Libraries ────────────────────────────────────────────
#include "DCMotor.h"
#include "Hbridge.h"
#include "MqttManager.h"
#include "QuadratureEncoder.h"
#include "Robotics.h"
#include "ServoStepper.h"
#include "SimpleGPIO.h"
#include "SimpleTimer.h"
#include "Stepper.h"
#include "WifiManager.h"

// ─── Timing ───────────────────────────────────────────────────────
float dt = 10000; // us — 10 ms control loop

// ─── Pin Definitions ──────────────────────────────────────────────

// Base stepper (ServoStepper — closed loop)
uint8_t BASE_DIR_PIN = 4;
uint8_t BASE_PWM_PIN = 5;
uint8_t BASE_PWM_CH  = 0;

// Z axis stepper (open loop)
uint8_t Z_DIR_PIN = 19;
uint8_t Z_PWM_PIN = 18;
uint8_t Z_PWM_CH  = 1;

// DC motor — arm (elbow joint)
uint8_t DC_PINS[2]  = {13, 14};
uint8_t DC_CH[2]    = {2, 3};
uint8_t ENC_PINS[2] = {25, 26};

// DC motor — wrist
uint8_t WRIST_DC_PINS[2]  = {23, 15};
uint8_t WRIST_DC_CH[2]    = {4, 5};
uint8_t WRIST_ENC_PINS[2] = {32, 33};

// Z-axis limit switch (active-low, internal pull-up)
#define Z_LIMIT_PIN  GPIO_NUM_27
#define Z_HOME_MM    190.0f   // physical Z position when limit switch fires

// Servo for gripper
uint8_t Servo_Pin = 16;
uint8_t Servo_CH  = 7;

// ─── LEDC Timer Configs ───────────────────────────────────────────
static TimerConfig STEPPER_TIMER_0{// Base stepper
                                   .timer          = LEDC_TIMER_0,
                                   .frequency      = 5000,
                                   .bit_resolution = LEDC_TIMER_12_BIT,
                                   .mode           = LEDC_LOW_SPEED_MODE};

static TimerConfig STEPPER_TIMER_1{// Z stepper
                                   .timer          = LEDC_TIMER_1,
                                   .frequency      = 4050,
                                   .bit_resolution = LEDC_TIMER_12_BIT,
                                   .mode           = LEDC_LOW_SPEED_MODE};

static TimerConfig DC_TIMER{// DC motors (shared)
                            .timer          = LEDC_TIMER_2,
                            .frequency      = 20000,
                            .bit_resolution = LEDC_TIMER_8_BIT,
                            .mode           = LEDC_HIGH_SPEED_MODE};

static TimerConfig Servo_TIMER{// Gripper servo
                               .timer          = LEDC_TIMER_3,
                               .frequency      = 50,
                               .bit_resolution = LEDC_TIMER_8_BIT,
                               .mode           = LEDC_HIGH_SPEED_MODE};

// ─── Peripheral Objects ───────────────────────────────────────────
SimpleTimer       Timer;
WifiManager       wifi;
MqttManager       mqtt;
ServoStepper      Base_Stepper;
Stepper           Z_Stepper;
HBridge           DC_Motor;
QuadratureEncoder Encoder_arm;
DCMotor           arm_motor;
DCMotor           wrist_motor;
Robotics          robot;
SimpleGPIO        Z_LimitSwitch;
SimplePWM         Servo;

// ─── PID Gains ────────────────────────────────────────────────────
float base_gains[3]      = {8.0f, 1.48f, 0.0f};
float z_gains[3]         = {10.0f, 0.0f, 0.0f};
float arm_pos_gains[3]   = {3.0f, 0.1f, 0.1f};
float arm_vel_gains[3]   = {2.0f, 0.5f, 0.05f};
float wrist_pos_gains[3] = {3.0f, 0.1f, 0.1f};
float wrist_vel_gains[3] = {2.0f, 0.5f, 0.05f};

// ─── Mechanical ───────────────────────────────────────────────────
#define Z_STEPS_PER_REV 1600u    // Z stepper microstepping setting

float BASE_RATIO  = 4.0f;              // motor° per base joint°
float Z_RATIO     = 1000.0f / 22.0f;   // motor° per mm of Z travel (leadscrew)
float ARM_RATIO   = 560.0f / 90.0f;    // motor° per elbow joint°
float WRIST_RATIO = 560.0f / 90.0f;    // motor° per wrist joint°

// SCARA link lengths (mm) — workspace: |L1-L2| ≤ p ≤ L1+L2
float scara_l1 = 150.0f;
float scara_l2 = 100.0f;

// ─── MQTT Protocol ────────────────────────────────────────────────
//
//   "0,base_deg,z_mm,elbow_deg,wrist_deg"   joint mode
//   "1,x,y,z,tool_angle_deg"                Cartesian (IK) mode
//   "2"                                     Z homing via limit switch
//   "3"                                     zero base/elbow/wrist at current pose
//
int mode      = -1;
int prev_mode = -1;

// Mode-0 commands (joint-space)
float cmd_base_deg  = 0.0f;
float cmd_z_mm      = 0.0f;
float cmd_elbow_deg = 0.0f;
float cmd_wrist_deg = 0.0f;

// Mode-1 commands (Cartesian) — defaults inside workspace (p = L1+L2-distance)
float cmd_x        = 150.0f;
float cmd_y        = 0.0f;
float cmd_z        = 0.0f;
float cmd_tool_deg = 0.0f;

// IK cache — NaN forces a solve on first entry
float last_cmd_x    = NAN;
float last_cmd_y    = NAN;
float last_cmd_z    = NAN;
float last_cmd_tool = NAN;

// Motor targets (joint-side, applied every tick once motors_active is true)
float target_base_deg  = 0.0f;
float target_z_mm      = 0.0f;
float target_elbow_deg = 0.0f;
float target_wrist_deg = 0.0f;

bool motors_active = false;  // suppresses motor commands until first valid input
bool z_homing      = false;

// ─── Network ──────────────────────────────────────────────────────
#define WIFI_SSID       "realme 11 Pro 5G"
#define WIFI_PASSWORD   "z57ek35n"
#define MQTT_BROKER_URI "mqtt://10.195.83.10:1883"
#define MQTT_CLIENT_ID  "ESP32_Client_01"
#define TOPIC_PUB       "esp32/status"
#define TOPIC_SUB       "esp32/commands"

// ─── Misc ─────────────────────────────────────────────────────────
char pub_buf[96];
int  pub_tick = 0;

#endif // __DEFINITIONS_H__
