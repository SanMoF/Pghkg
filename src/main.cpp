#include "definitions.h"

static void IRAM_ATTR timerinterrupt(void *arg)
{
    Timer.setInterrupt();
}

static void mqtt_task(void *pvParameters)
{
    while (1)
    {
        const char *raw = mqtt.subscribe();

        if (raw != nullptr && raw[0] != '\0')
        {
            char buf[64];
            strncpy(buf, raw, sizeof(buf) - 1);
            buf[sizeof(buf) - 1] = '\0';

            char *token = strtok(buf, ",");
            if (token == nullptr)
            {
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }

            mode = atoi(token);

            if (mode == 0)
            {
                token = strtok(nullptr, ","); if (token) cmd_base_deg  = atof(token);
                token = strtok(nullptr, ","); if (token) cmd_z_mm      = atof(token);
                token = strtok(nullptr, ","); if (token) cmd_elbow_deg = atof(token);
                token = strtok(nullptr, ","); if (token) cmd_wrist_deg = atof(token);
            }
            else if (mode == 1)
            {
                token = strtok(nullptr, ","); if (token) cmd_x        = atof(token);
                token = strtok(nullptr, ","); if (token) cmd_y        = atof(token);
                token = strtok(nullptr, ","); if (token) cmd_z        = atof(token);
                token = strtok(nullptr, ","); if (token) cmd_tool_deg = atof(token);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

extern "C" void app_main()
{
    esp_task_wdt_deinit();

    robot.setup(scara_l1, scara_l2);

    Timer.setup(timerinterrupt, "Main_timer");
    Timer.startPeriodic(dt);

    Base_Stepper.setup(BASE_DIR_PIN, BASE_PWM_PIN, BASE_PWM_CH,
                       &STEPPER_TIMER_0, base_gains, dt);

    Z_Stepper.setup(Z_DIR_PIN, Z_PWM_PIN, Z_PWM_CH,
                    &STEPPER_TIMER_1, Z_STEPS_PER_REV,
                    z_gains[0], z_gains[1], z_gains[2], (uint32_t)dt);

    Z_LimitSwitch.setup(Z_LIMIT_PIN, GPI, GPIO_PULLUP_ONLY);

    arm_motor.setup(DC_PINS, DC_CH, ENC_PINS, &DC_TIMER,
                    arm_vel_gains, arm_pos_gains, dt);
    arm_motor.setMode(DCMotorMode::POSITION);

    wrist_motor.setup(WRIST_DC_PINS, WRIST_DC_CH, WRIST_ENC_PINS, &DC_TIMER,
                      wrist_vel_gains, wrist_pos_gains, dt);
    wrist_motor.setMode(DCMotorMode::POSITION);

    Servo.setup(Servo_Pin, Servo_CH, &Servo_TIMER);

    wifi.setup(WIFI_SSID, WIFI_PASSWORD);
    mqtt.setup(MQTT_BROKER_URI, MQTT_CLIENT_ID, TOPIC_SUB);
    mqtt.publish(TOPIC_PUB, "ESP32 online");

    xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 5, NULL);

    while (1)
    {
        if (Timer.interruptAvailable())
        {
            // ── Z homing overrides normal Z control ─────────────────
            if (z_homing)
            {
                if (Z_LimitSwitch.get() == 0)
                {
                    Z_Stepper.forceStop();
                    Z_Stepper.resetPosition();
                    target_z_mm = 0.0f;          // hold at home
                    z_homing    = false;
                    mqtt.publish(TOPIC_PUB, "Z:homed");
                    printf("Z homing complete — position zeroed\n");
                }
                else
                {
                    Z_Stepper.update();
                }
            }

            // ── Mode handling: updates target_* state ───────────────
            switch (mode)
            {
            case 0:
                target_base_deg  = cmd_base_deg;
                target_z_mm      = cmd_z_mm;
                target_elbow_deg = cmd_elbow_deg;
                target_wrist_deg = cmd_wrist_deg;
                motors_active    = true;
                break;

            case 1:
            {
                // Re-solve IK only on cmd change or mode entry — avoids
                // mid-motion elbow-flip and saves CPU.
                bool cmd_changed = (cmd_x != last_cmd_x) ||
                                   (cmd_y != last_cmd_y) ||
                                   (cmd_z != last_cmd_z) ||
                                   (cmd_tool_deg != last_cmd_tool);

                if (prev_mode != 1 || cmd_changed)
                {
                    float cur_base  = Base_Stepper._encoder.getAccumulatedAngleDeg() / BASE_RATIO;
                    float cur_elbow = arm_motor.getPosition() / ARM_RATIO;
                    float cur_z_mm  = (Z_Stepper.getPosition() * 360.0f /
                                       (float)Z_Stepper.stepsPerRev()) / Z_RATIO;
                    float cur_wrist = wrist_motor.getPosition() / WRIST_RATIO;

                    IKSolution sol;
                    bool reachable = robot.solveIK(cmd_x, cmd_y, cmd_z, cmd_tool_deg,
                                                   cur_base, cur_elbow, cur_z_mm, cur_wrist,
                                                   sol);
                    if (reachable)
                    {
                        target_base_deg  = sol.theta1 * 57.29577951f;
                        target_elbow_deg = sol.theta2 * 57.29577951f;
                        target_z_mm      = sol.z;
                        target_wrist_deg = sol.theta4 * 57.29577951f;

                        last_cmd_x    = cmd_x;
                        last_cmd_y    = cmd_y;
                        last_cmd_z    = cmd_z;
                        last_cmd_tool = cmd_tool_deg;
                        motors_active = true;
                    }
                    else
                    {
                        printf("IK unreachable: (%.2f,%.2f,%.2f)\n", cmd_x, cmd_y, cmd_z);
                    }
                }
                break;
            }

            case 2:  // trigger Z homing
                z_homing = true;
                Z_Stepper.goToAngle(-99999.0f * Z_RATIO, 2000);
                mode = -1;
                break;

            case 3:  // declare current pose as zero for base/elbow/wrist
                Base_Stepper._encoder.resetAccumulatedAngle();
                arm_motor.zeroPosition();
                wrist_motor.zeroPosition();
                target_base_deg  = 0.0f;
                target_elbow_deg = 0.0f;
                target_wrist_deg = 0.0f;
                last_cmd_x = last_cmd_y = last_cmd_z = last_cmd_tool = NAN;
                motors_active = true;
                mqtt.publish(TOPIC_PUB, "JOINTS:zeroed");
                mode = -1;
                break;

            default:
                break;
            }

            prev_mode = mode;

            // ── Apply targets every tick once we have a valid command ─
            if (motors_active)
            {
                Base_Stepper.goToAngle(target_base_deg * BASE_RATIO);
                arm_motor.setTargetPosition(target_elbow_deg * ARM_RATIO);
                wrist_motor.setTargetPosition(target_wrist_deg * WRIST_RATIO);
                arm_motor.update();
                wrist_motor.update();

                if (!z_homing)   // homing block already drives Z
                {
                    Z_Stepper.goToAngle(target_z_mm * Z_RATIO, 4000);
                    Z_Stepper.update();
                }
            }

            // ── Telemetry at 10 Hz ─────────────────────────────────
            if (++pub_tick >= 10)
            {
                pub_tick = 0;

                float base_deg  = Base_Stepper._encoder.getAccumulatedAngleDeg() / BASE_RATIO;
                float z_mm      = (Z_Stepper.getPosition() * 360.0f /
                                   (float)Z_Stepper.stepsPerRev()) / Z_RATIO;
                float elbow_deg = arm_motor.getPosition() / ARM_RATIO;
                float wrist_deg = wrist_motor.getPosition() / WRIST_RATIO;

                EndEffectorPose pose = robot.getEndEffectorPosition(
                    base_deg, elbow_deg, z_mm, wrist_deg);

                snprintf(pub_buf, sizeof(pub_buf),
                         "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                         base_deg, z_mm, elbow_deg, wrist_deg,
                         pose.x, pose.y, pose.z);
                mqtt.publish(TOPIC_PUB, pub_buf);
            }
        }
    }
}
