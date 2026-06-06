#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

// ─── External Libraries ───────────────────────────────────────────
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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
                               .bit_resolution = LEDC_TIMER_10_BIT,
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

// Base motor turns the opposite handedness to the IK convention (atan2 is
// CCW-positive; the base hardware is CW-positive), so the sign is negative.
// It flips both the command (joint→motor) and feedback (motor→joint) together,
// keeping the whole base chain in one consistent frame.
float BASE_RATIO  = -4.0f;             // motor° per base joint° (inverted)
float Z_RATIO     = 1000.0f / 22.0f;   // motor° per mm of Z travel (leadscrew)
float ARM_RATIO   = 560.0f / 90.0f;    // motor° per elbow joint°
// Wrist motor rotational sense (sign) was flipped so a positive joint angle
// (IK convention) drives the motor the correct way. The sign keeps both the
// command (joint→motor) and feedback (motor→joint) conversions consistent.
float WRIST_RATIO = 560.0f / 90.0f * 1.5f * 0.5f;   // motor° per wrist joint°

// SCARA link lengths (mm) — workspace: |L1-L2| ≤ p ≤ L1+L2
float scara_l1 = 150.0f;
float scara_l2 = 100.0f;

// Vertical (Z) geometry — see Robotics.h for the full model.
//   ARM_PLANE_HOME : arm-plane height when Z is homed (top limit)
//   TCP_Z_DROP     : fixed vertical drop from the wrist down to the TCP
//   home TCP height = ARM_PLANE_HOME - TCP_Z_DROP = 340 - 130 = 210 mm
float ARM_PLANE_HOME = 215.0f;
float TCP_Z_DROP     = 130.0f;

// ─── MQTT Protocol ────────────────────────────────────────────────
//
//   "0,base_deg,z_mm,elbow_deg,wrist_deg"   joint mode
//   "1,x,y,z,tool_angle_deg"                Cartesian (IK) mode
//   "2"                                     Z homing via limit switch
//   "3"                                     zero base/elbow/wrist at current pose
//   "4"                                     run hardcoded pick & place list
//   "4,px,py,pz,qx,qy,qz[,close,open]"      single pick→place over MQTT
//
//   Published telemetry (esp32/status), 17 comma-separated fields @ 10 Hz:
//     base_deg, z_mm, elbow_deg, wrist_deg,           joint positions
//     x, y, z,                                        TCP position (mm)
//     base_dps, z_mmps, elbow_dps, wrist_dps,         joint speeds
//     vx, vy, vz, omega,                              TCP velocity (mm/s, deg/s)
//     gripper_pwm                                     gripper state (servo duty)
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
float cmd_servo_pwm = 0.0f;   // duty cycle del servo (0–100 por ejemplo)


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

// ─── Pick & Place (mode 4) ────────────────────────────────────────
// Each row is a TCP target (x, y, z) in mm. Place rows sit at z ≈ 0.
// For every point the arm first moves PP_APPROACH_MM above it (approach),
// descends, actuates the gripper, then retreats back up — keeping the
// horizontal moves clear of the table. Add/remove rows freely; pick_pts
// and place_pts must stay the same length (PP_COUNT).
#define PP_COUNT 2
float pick_pts[PP_COUNT][3] = {
    {200.0f, 100.0f, 100.0f},
    {150.0f, -80.0f,  90.0f},
};
float place_pts[PP_COUNT][3] = {
    {300.0f,   2.0f,   2.0f},
    {250.0f, -50.0f,   0.0f},
};

float PP_APPROACH_MM = 30.0f;  // safe height above each point (mm)
float pp_tool_deg    = 0.0f;   // tool/wrist orientation held through the run

// Gripper servo duty (empirically 4–12). Tune to your gripper.
float gripper_open_pwm  = 12.0f;  // release
float gripper_close_pwm =  6.0f;  // grip

// Arrival tolerances and gripper dwell (ticks @ 10 ms loop)
float PP_ANG_TOL = 2.0f;   // joint arrival tolerance (deg)
float PP_Z_TOL   = 2.0f;   // Z arrival tolerance (mm)
#define PP_GRIP_TICKS 50   // dwell so the servo finishes moving (~500 ms)

// A move sub-state advances once the joints settle OR this many ticks pass, so
// a joint that physically can't reach its target (e.g. wrist) never deadlocks
// the sequence. 250 ticks ≈ 2.5 s.
#define PP_SETTLE_TIMEOUT 250
int pp_settle = 0;         // ticks spent in the current move sub-state

// Pick & place sub-state machine
enum PPState
{
    PP_APPROACH_PICK, // move above the pick point
    PP_DESCEND_PICK,  // lower onto the pick point
    PP_GRIP_CLOSE,    // close gripper (dwell)
    PP_LIFT_PICK,     // retreat back up
    PP_APPROACH_PLACE,// move above the place point
    PP_DESCEND_PLACE, // lower onto the place point
    PP_GRIP_OPEN,     // open gripper (dwell)
    PP_LIFT_PLACE,    // retreat, then advance to next pair
    PP_DONE
};
int  pp_run_count = PP_COUNT; // pairs to run this pass (1 when sent over MQTT)
bool pp_restart   = false;    // a new mode-4 command asks to restart the run
int  pp_index     = 0;
int  pp_state     = PP_APPROACH_PICK;
bool pp_new_state = true;   // true on first tick in a sub-state → solve IK once
bool pp_reachable = false;  // last IK solve for the current sub-state succeeded
int  pp_dwell     = 0;      // gripper dwell counter

// ─── Network ──────────────────────────────────────────────────────
#define WIFI_SSID       "realme 11 Pro 5G"
#define WIFI_PASSWORD   "z57ek35n"
#define MQTT_BROKER_URI "mqtt://10.195.83.10:1883"
#define MQTT_CLIENT_ID  "ESP32_Client_01"
#define TOPIC_PUB       "esp32/status"
#define TOPIC_SUB       "esp32/commands"

// ─── Misc ─────────────────────────────────────────────────────────
char pub_buf[192];
int  pub_tick = 0;

// Previous joint positions, for differentiating into actual joint speeds in
// the telemetry block. have_prev_joint is false until the first sample so the
// first reported speed is 0 instead of a huge spike.
float prev_base_deg    = 0.0f;
float prev_z_mm        = 0.0f;
float prev_elbow_deg   = 0.0f;
float prev_wrist_deg   = 0.0f;
bool  have_prev_joint  = false;

// ─── Comms decoupling ─────────────────────────────────────────────
// The control loop must never block on the network. Instead of publishing
// inline, it enqueues short status strings here; a separate comms task drains
// the queue and does the (potentially blocking) esp_mqtt publish off the
// real-time core.
#define MQTT_MSG_LEN      192
#define MQTT_TX_QUEUE_LEN 8
QueueHandle_t mqtt_tx_q = nullptr;

#endif // __DEFINITIONS_H__
