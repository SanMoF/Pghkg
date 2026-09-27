#include "definitions.h"

// ── ISR handlers — one per SimpleTimer ──────────────────────────────
static void IRAM_ATTR ctrlISR(void *arg)  { ctrlTimer.setInterrupt();  }
static void IRAM_ATTR commISR(void *arg)  { commTimer.setInterrupt();  }
static void IRAM_ATTR telemISR(void *arg) { telemTimer.setInterrupt(); }

// ── UART command protocol ───────────────────────────────────────────
//
//   MODE:0            switch to manual speed control
//   MODE:1            switch to PID balance control
//   M1:<pct>          motor 1 speed, 0..100 %   (mode 0 only, no reverse — ESCs are unidirectional)
//   M2:<pct>          motor 2 speed, 0..100 %   (mode 0 only, no reverse — ESCs are unidirectional)
//   SET:<deg>         balance setpoint (target pitch, deg)
//   PID:<kp>,<ki>,<kd> balance PID gains
//   BASE:<pct>        balance base thrust, 0..100 % — both motors mix
//                      around this; the PID output adds to one side and
//                      subtracts from the other (mode 1 only)
//   MAXPCT:<pct>      throttle ceiling, 0..100 % — caps both ESCs'
//                      setThrottlePercent() regardless of mode
//   STOP              zero both motors and force mode 0 (safety)
//
// Example (type into the PlatformIO serial monitor, one per line):
//   MODE:0
//   M1:30
//   M2:30
//   MODE:1
//   SET:0.0
//   PID:1.0,0.5,0.8
//   BASE:40
//
static void processUartLine(char *line)
{
    // Trim a leading/trailing space; commands are otherwise plain text.
    while (*line == ' ')
        line++;

    char *colon = strchr(line, ':');
    char *args  = nullptr;
    if (colon)
    {
        *colon = '\0';
        args   = colon + 1;
    }

    if (strcmp(line, "MODE") == 0 && args)
    {
        int m = atoi(args);
        if (m == MODE_SPEED)
        {
            motor1SpeedPct = 0.0f;
            motor2SpeedPct = 0.0f;
            esc1.stop();
            esc2.stop();
            controlMode = MODE_SPEED;
            printf("-> MODE SPEED\n");
        }
        else if (m == MODE_BALANCE)
        {
            balancePID.reset();
            controlMode = MODE_BALANCE;
            printf("-> MODE BALANCE\n");
        }
        else
        {
            printf("-> MODE: invalid value '%s' (use 0 or 1)\n", args);
        }
    }
    else if (strcmp(line, "M1") == 0 && args)
    {
        if (controlMode != MODE_SPEED)
            printf("-> M1 ignored: not in MODE:0\n");
        else
        {
            motor1SpeedPct = atof(args);
            printf("-> M1=%.1f%%\n", motor1SpeedPct);
        }
    }
    else if (strcmp(line, "M2") == 0 && args)
    {
        if (controlMode != MODE_SPEED)
            printf("-> M2 ignored: not in MODE:0\n");
        else
        {
            motor2SpeedPct = atof(args);
            printf("-> M2=%.1f%%\n", motor2SpeedPct);
        }
    }
    else if (strcmp(line, "SET") == 0 && args)
    {
        balanceSetpointDeg = atof(args);
        printf("-> SETPOINT=%.2f deg\n", balanceSetpointDeg);
    }
    else if (strcmp(line, "PID") == 0 && args)
    {
        float kp = 0.0f, ki = 0.0f, kd = 0.0f;
        char *tok = strtok(args, ",");
        if (tok) kp = atof(tok);
        tok = strtok(nullptr, ",");
        if (tok) ki = atof(tok);
        tok = strtok(nullptr, ",");
        if (tok) kd = atof(tok);

        balanceGains[0] = kp;
        balanceGains[1] = ki;
        balanceGains[2] = kd;
        balancePID.setup(balanceGains, (float)CTRL_DT_US);
        balancePID.setULimit(BALANCE_OUTPUT_LIMIT_PCT);
        printf("-> PID Kp=%.3f Ki=%.3f Kd=%.3f\n", kp, ki, kd);
    }
    else if (strcmp(line, "BASE") == 0 && args)
    {
        balanceBasePct = atof(args);
        if (balanceBasePct < 0.0f)   balanceBasePct = 0.0f;
        if (balanceBasePct > 100.0f) balanceBasePct = 100.0f;
        printf("-> BASE=%.1f%%\n", balanceBasePct);
    }
    else if (strcmp(line, "MAXPCT") == 0 && args)
    {
        escMaxThrottlePct = atof(args);
        if (escMaxThrottlePct < 0.0f)   escMaxThrottlePct = 0.0f;
        if (escMaxThrottlePct > 100.0f) escMaxThrottlePct = 100.0f;
        esc1.setMaxThrottlePercent(escMaxThrottlePct);
        esc2.setMaxThrottlePercent(escMaxThrottlePct);
        printf("-> MAXPCT=%.1f%%\n", escMaxThrottlePct);
    }
    else if (strcmp(line, "STOP") == 0)
    {
        motor1SpeedPct = 0.0f;
        motor2SpeedPct = 0.0f;
        esc1.stop();
        esc2.stop();
        controlMode = MODE_SPEED;
        printf("-> STOP\n");
    }
    else
    {
        printf("-> Unknown command: '%s'\n", line);
    }
}

// Feed raw UART bytes into a line buffer and dispatch complete lines.
// Runs off the 20 ms comm timer — never in the 5 ms control guard.
static void pollUartCommands()
{
    char chunk[33];
    int  n;
    while (console.available() > 0 && (n = console.read(chunk, sizeof(chunk) - 1)) > 0)
    {
        for (int i = 0; i < n; i++)
        {
            char c = chunk[i];
            if (c == '\n' || c == '\r')
            {
                if (uartLineLen > 0)
                {
                    uartLineBuf[uartLineLen] = '\0';
                    processUartLine(uartLineBuf);
                    uartLineLen = 0;
                }
            }
            else if (uartLineLen < UART_LINE_MAX - 1)
            {
                uartLineBuf[uartLineLen++] = c;
            }
        }
    }
}

extern "C" void app_main()
{
    esp_task_wdt_deinit(); // ALWAYS first line

    // ── Setup: peripherals and comms first, timers LAST ─────────────
    // (console/UART0 is already installed by SimpleUART's constructor above)

    esc1.setup(ESC1_PIN, ESC1_CH, &ESC_TIMER);
    esc2.setup(ESC2_PIN, ESC2_CH, &ESC_TIMER);
    // Deadband is per-ESC, NOT shared: esc1 was jumping to full throttle at
    // low commanded pct because it was reusing esc2's 24% calibration, which
    // doesn't clear esc1's own spin-up threshold cleanly on its specific
    // ESC/motor pair. Re-run the ramp-up-from-0 calibration on EACH motor
    // separately (MODE:0, then M1/M2 from 0 upward until it just starts
    // turning) and set its own threshold below.
    #define ESC1_DEADBAND_PCT 10.0f // TODO: recalibrate on esc1's motor
    #define ESC2_DEADBAND_PCT 24.0f
    esc1.setDeadbandPercent(ESC1_DEADBAND_PCT);
    esc2.setDeadbandPercent(ESC2_DEADBAND_PCT);
    esc1.setMaxThrottlePercent(escMaxThrottlePct);
    esc2.setMaxThrottlePercent(escMaxThrottlePct);
    esc1.arm();
    esc2.arm();
    // Hold the stop pulse so both ESCs finish arming before any throttle
    // command is possible. One-time wait — setup only, never in the loop.
    vTaskDelay(pdMS_TO_TICKS(2500));

    esp_err_t imu_err = imu.begin();
    imuAvailable = (imu_err == ESP_OK);
    if (imuAvailable)
        imu.calibrateGyro(); // robot must be held still during boot
    else
        printf("WARNING: MPU6050 not detected — balance mode will not work, "
               "IMU reads will be skipped\n");

    balancePID.setup(balanceGains, (float)CTRL_DT_US);
    balancePID.setULimit(BALANCE_OUTPUT_LIMIT_PCT);

    printf("BLDC balance/speed controller ready.\n");
    printf("Commands: MODE:0|1  M1:<pct>  M2:<pct>  SET:<deg>  PID:<kp>,<ki>,<kd>  "
           "MAXPCT:<pct>  STOP\n");

    ctrlTimer.setup(ctrlISR, "CtrlTimer");
    commTimer.setup(commISR, "CommTimer");
    telemTimer.setup(telemISR, "TelemTimer");

    ctrlTimer.startPeriodic(CTRL_DT_US);
    commTimer.startPeriodic(COMM_DT_US);
    telemTimer.startPeriodic(TELEM_DT_US);

    // ── Loop — no delays, only SimpleTimer guards ────────────────────
    while (1)
    {
        if (ctrlTimer.interruptAvailable())
        {
            if (controlMode == MODE_SPEED)
            {
                esc1.setThrottlePercent(motor1SpeedPct);
                esc2.setThrottlePercent(motor2SpeedPct);
            }
            // IMU is updated every control tick regardless of mode, so pitch
            // stays fresh for debugging even outside MODE_BALANCE.
            if (imuAvailable)
                imuAvailable = (imu.update(CTRL_DT_S) == ESP_OK);

            if (controlMode == MODE_BALANCE && imuAvailable)
            {
                float pitch = imu.getPitchDeg();
                float error = balanceSetpointDeg - pitch;
                balanceOutputPct = balancePID.computedU(error);

                // Near setpoint, drop the shared base way down instead of
                // holding full base thrust while balanced — otherwise the
                // motors keep spinning hard on tiny noise once on target.
                float effectiveBasePct = balanceBasePct;
                if (fabsf(error) < BALANCE_NEAR_SETPOINT_DEG)
                    effectiveBasePct *= BALANCE_NEAR_SETPOINT_SCALE;

                // Seesaw balance: differential thrust around a shared base,
                // one side up and the other down — NOT the same value on
                // both (that only changes total lift, never the tilt).
                // setThrottlePercent() clamps each side to [0, 100] on its
                // own, so this needs no extra clamping here.
                esc1.setThrottlePercent(effectiveBasePct + balanceOutputPct);
                esc2.setThrottlePercent(effectiveBasePct - balanceOutputPct);
            }
        }

        if (commTimer.interruptAvailable())
        {
            pollUartCommands();
        }

        if (telemTimer.interruptAvailable())
        {
            if (controlMode == MODE_SPEED)
            {
                printf("[SPEED] M1=%.1f%% M2=%.1f%%\n", motor1SpeedPct, motor2SpeedPct);
            }
            else
            {
                printf("[BALANCE] pitch=%.2f set=%.2f base=%.1f%% out=%.2f%% -> "
                       "M1=%.1f%% M2=%.1f%% (Kp=%.2f Ki=%.2f Kd=%.2f)\n",
                       imu.getPitchDeg(), balanceSetpointDeg, balanceBasePct, balanceOutputPct,
                       balanceBasePct + balanceOutputPct, balanceBasePct - balanceOutputPct,
                       balanceGains[0], balanceGains[1], balanceGains[2]);
            }

            // Raw IMU readout for debugging the sensor itself, independent
            // of control mode. One I2C read at 300 ms — never in the 5 ms
            // control guard (failure mode #5). Skipped entirely once the
            // sensor is known absent, so a missing IMU logs one WARNING at
            // boot instead of spamming an error every telemetry tick.
            if (imuAvailable)
            {
                int16_t accelRaw[3], gyroRaw[3];
                if (imu.readRaw(accelRaw, gyroRaw) == ESP_OK)
                {
                    printf("[IMU] pitch=%.2f ax=%.3f ay=%.3f az=%.3f gx=%.2f gy=%.2f gz=%.2f\n",
                           imu.getPitchDeg(),
                           accelRaw[0] / MPU6050_ACCEL_LSB_PER_G,
                           accelRaw[1] / MPU6050_ACCEL_LSB_PER_G,
                           accelRaw[2] / MPU6050_ACCEL_LSB_PER_G,
                           gyroRaw[0] / MPU6050_GYRO_LSB_PER_DPS,
                           gyroRaw[1] / MPU6050_GYRO_LSB_PER_DPS,
                           gyroRaw[2] / MPU6050_GYRO_LSB_PER_DPS);
                }
                else
                {
                    printf("[IMU] read error — sensor lost, disabling further reads\n");
                    imuAvailable = false;
                }
            }
        }
    }
}
