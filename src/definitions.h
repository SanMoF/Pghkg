#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

// ─── External Libraries ───────────────────────────────────────────
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ─── Project Libraries ────────────────────────────────────────────
#include "SimpleTimer.h"
#include "SimpleUART.h"
#include "SimplePWM.h"
#include "BLDC_ESC.h"
#include "MPU6050.h"
#include "PID_CAYETANO.h"

// ─── Pin Definitions ──────────────────────────────────────────────

// BLDC ESC signal pins (50 Hz servo-style PWM, one LEDC channel each)
#define ESC1_PIN     GPIO_NUM_18
#define ESC1_CH      0
#define ESC2_PIN     GPIO_NUM_19
#define ESC2_CH      1

// MPU6050 I2C pins (balance sensor)
// Backup branch: this is the pre-BMI270 driver, kept ready in case the
// BMI270 module fails physically. Same bus/pins as the BMI270 build.
#define IMU_SDA_PIN  GPIO_NUM_21
#define IMU_SCL_PIN  GPIO_NUM_22

// UART used for the control console (default USB serial monitor port)
#define CONSOLE_BAUD 115200

// ─── Timing ───────────────────────────────────────────────────────
#define CTRL_DT_US   5000u      // 5 ms  — balance/motor control loop
#define IMU_RECOVER_PERIOD_MS 1000    // retry interval to re-init the IMU after it was lost mid-run
#define IMU_MAX_CONSECUTIVE_ERRORS 10 // failed IMU reads in a row (10 x 5 ms = 50 ms) before the IMU is declared lost
#define CTRL_DT_S    0.005f
#define COMM_DT_US   20000u     // 20 ms — UART command polling
#define TELEM_DT_US  300000u    // 300 ms — status printout

// ─── LEDC Timer Config ────────────────────────────────────────────
// Both ESCs share one 50 Hz LEDC timer (matches the RC-ESC PWM convention),
// one channel each — same pattern as the project's Servo_TIMER.
static TimerConfig ESC_TIMER{
    .timer          = LEDC_TIMER_0,
    .frequency      = 50,
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode           = LEDC_HIGH_SPEED_MODE};

// ─── Peripheral Objects ───────────────────────────────────────────
SimpleTimer  ctrlTimer;
SimpleTimer  commTimer;
SimpleTimer  telemTimer;
SimpleUART   console(CONSOLE_BAUD, UART_NUM_0);
BLDC_ESC     esc1;
BLDC_ESC     esc2;
MPU6050      imu(I2C_NUM_0, IMU_SDA_PIN, IMU_SCL_PIN, 100000);
PID_CAYETANO balancePID;
bool         imuAvailable = false; // set once in setup() after imu.begin()
bool         imuLost = false;      // true once the IMU dropped out mid-run (vs. absent at boot)
int64_t      imuLostAtUs = 0;      // esp_timer time of the last loss/recovery attempt
int          imuErrorCount = 0;    // consecutive failed imu.update() calls

// ─── Control Modes ─────────────────────────────────────────────────
enum ControlMode
{
    MODE_SPEED   = 0, // manual per-motor speed control, in %
    MODE_BALANCE = 1  // PID balance control, driven by the MPU6050 pitch
};
int controlMode = MODE_SPEED;

// ── Mode 0: manual speed (%). Range [-100, 100]; sign = direction. ──
float motor1SpeedPct = 0.0f;
float motor2SpeedPct = 0.0f;

// ── Mode 1: balance PID ─────────────────────────────────────────────
// This is a seesaw/rocker balance: one motor at each end, correcting tilt
// needs DIFFERENTIAL thrust (one side up, the other down from a shared
// base), not the same throttle on both. ESCs have no reverse, so "down"
// only works down to 0% — the base has to sit above 0 so the falling side
// still has room to actually decrease.
//   esc1 = balanceBasePct + balanceOutputPct
//   esc2 = balanceBasePct - balanceOutputPct
float balanceSetpointDeg = 0.0f;         // target pitch, deg
float balanceGains[3]     = {1.0f, 0.5f, 0.01f}; // Kp, Ki, Kd
float balanceOutputPct    = 0.0f;        // last PID output, for telemetry
// Must stay under escMaxThrottlePct with room for +-BALANCE_OUTPUT_LIMIT_PCT,
// or setThrottlePercent()'s ceiling clamp pins BOTH sides to the same value
// for the whole PID output range — motors stop responding to angle changes
// entirely (was 40%, clamped flat against the 30% ceiling below).
float balanceBasePct      = 20.0f;       // shared thrust both sides mix around
#define BALANCE_OUTPUT_LIMIT_PCT 10.0f
// Below this angle error, most of the wobble is noise, not real tilt — cut
// the shared base thrust way down so the robot settles instead of buzzing
// the motors at full base speed while sitting on target.
#define BALANCE_NEAR_SETPOINT_DEG   1.0f
#define BALANCE_NEAR_SETPOINT_SCALE 0.25f

// ── Runtime throttle ceiling ─────────────────────────────────────────
// Caps both ESCs' setThrottlePercent() so nothing beyond this ever reaches
// the motors. 30% was found on the bench to be a safe ceiling for the ESC
// that was jumping to full throttle at low commanded pct. Adjustable at
// runtime (MAXPCT:<pct> over UART / the HMI) instead of baked in, since the
// right value depends on hardware behavior discovered by testing, not a
// fixed calibration like the deadband.
float escMaxThrottlePct = 30.0f;

// ── UART line-command buffer ────────────────────────────────────────
#define UART_LINE_MAX 96
char   uartLineBuf[UART_LINE_MAX];
size_t uartLineLen = 0;

#endif // __DEFINITIONS_H__
