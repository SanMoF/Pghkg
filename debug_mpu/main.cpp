// Standalone MPU6050 debug program.
//
// Independent of the balance firmware (src/main.cpp / src/definitions.h) —
// this env compiles ONLY this file (see [env:mpu_debug] in platformio.ini,
// src_dir = debug_mpu), while still reusing the real lib/MPU6050 driver
// (PlatformIO auto-collects lib/ for every env, no copying needed).
//
// Purpose: bench-check wiring/I2C address, watch raw accel/gyro, and
// compare the filtered pitch against the accel-only pitch so the sign/
// orientation of the sensor can be confirmed by hand-tilting the robot.
// This can't be verified from code alone — only by reading these prints
// while physically tilting the board.
//
// Flash & watch:
//   pio run -e mpu_debug -t upload
//   pio device monitor

#include <math.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_task_wdt.h"

#include "MPU6050.h"

// Same physical pins as src/definitions.h (MPU_SDA_PIN / MPU_SCL_PIN),
// hardcoded here so this program has zero dependency on src/.
static const gpio_num_t DBG_MPU_SDA_PIN = GPIO_NUM_21;
static const gpio_num_t DBG_MPU_SCL_PIN = GPIO_NUM_22;

static const uint32_t LOOP_DT_MS = 100;                    // ~100 ms debug loop
static const float    LOOP_DT_S  = LOOP_DT_MS / 1000.0f;   // must match imu.update(dt)

extern "C" void app_main()
{
    esp_task_wdt_deinit(); // ALWAYS first line

    MPU6050 imu(I2C_NUM_0, DBG_MPU_SDA_PIN, DBG_MPU_SCL_PIN);

    printf("\n=== MPU6050 standalone debug ===\n");
    printf("SDA=%d SCL=%d\n", (int)DBG_MPU_SDA_PIN, (int)DBG_MPU_SCL_PIN);

    esp_err_t err = imu.begin();
    if (err != ESP_OK) {
        // begin() already printed WHO_AM_I and ran scanBus() internally on
        // failure — nothing to duplicate here.
        printf("[DEBUG] imu.begin() failed: %s. Check wiring/address above, "
               "then reset the board.\n", esp_err_to_name(err));
        // Keep the task alive (idempotent) instead of returning, so the
        // watchdog / boot loop doesn't spam resets while the user is
        // rewiring on the bench.
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    printf("[DEBUG] MPU6050 detected OK. Hold the robot COMPLETELY STILL for "
           "gyro calibration...\n");
    imu.calibrateGyro();
    printf("[DEBUG] Calibration done. You may now tilt the robot by hand.\n");
    printf("[DEBUG] ax,ay,az in g | gx,gy,gz in deg/s | pitch_accel vs pitch_filt in deg\n\n");

    while (1) {
        int16_t accelRaw[3], gyroRaw[3];
        esp_err_t rerr = imu.readRaw(accelRaw, gyroRaw);
        if (rerr != ESP_OK) {
            printf("[DEBUG] readRaw error: %s\n", esp_err_to_name(rerr));
        } else {
            float ax = accelRaw[0] / MPU6050_ACCEL_LSB_PER_G;
            float ay = accelRaw[1] / MPU6050_ACCEL_LSB_PER_G;
            float az = accelRaw[2] / MPU6050_ACCEL_LSB_PER_G;
            float gx = gyroRaw[0] / MPU6050_GYRO_LSB_PER_DPS;
            float gy = gyroRaw[1] / MPU6050_GYRO_LSB_PER_DPS;
            float gz = gyroRaw[2] / MPU6050_GYRO_LSB_PER_DPS;

            // Same accel-only formula update() uses internally (ay/az — the
            // forward/back balance axis, confirmed by hand-tilting; ax/az
            // tracks left/right instead), computed here again purely for
            // the side-by-side sign/orientation check.
            float pitchAccelOnly = atan2f(-ay, az) * 180.0f / (float)M_PI;

            esp_err_t uerr = imu.update(LOOP_DT_S);
            if (uerr != ESP_OK) {
                printf("[DEBUG] update error: %s\n", esp_err_to_name(uerr));
            } else {
                printf("accel(g): ax=%.3f ay=%.3f az=%.3f | gyro(dps): gx=%.2f gy=%.2f gz=%.2f "
                       "| pitch_accel=%.2f pitch_filt=%.2f\n",
                       ax, ay, az, gx, gy, gz, pitchAccelOnly, imu.getPitchDeg());
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_DT_MS));
    }
}
