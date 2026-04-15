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
                token = strtok(nullptr, ",");
                if (token)
                    theta1 = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    theta2 = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    theta3 = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    theta4 = atof(token);
            }
            else if (mode == 1)
            {
                token = strtok(nullptr, ",");
                if (token)
                    cmd_x = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    cmd_y = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    cmd_z = atof(token);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

extern "C" void app_main()
{
    esp_task_wdt_deinit();

    robot.setup(scara_l1, scara_l2);

    // Timers Setups
    Timer.setup(timerinterrupt, "Main_timer");
    Timer.startPeriodic(dt);
    // Motors Setup
    Base_Stepper.setup(BASE_DIR_PIN, BASE_PWM_PIN, BASE_PWM_CH,
                       &STEPPER_TIMER_0, base_gains, dt);

    Z_Stepper.setup(Z_DIR_PIN, Z_PWM_PIN, Z_PWM_CH,
                    &STEPPER_TIMER_1, /*steps_per_rev=*/1600,
                    z_gains[0], z_gains[1], z_gains[2], (uint32_t)dt);
    Z_LimitSwitch.setup(Z_LIMIT_PIN, GPI, GPIO_PULLUP_ONLY);
    arm_motor.setup(DC_PINS, DC_CH, ENC_PINS, &DC_TIMER,
                    arm_vel_gains, arm_pos_gains, dt);
    arm_motor.setMode(DCMotorMode::POSITION);
    // Wifi and Mqtt setups
    wifi.setup(WIFI_SSID, WIFI_PASSWORD);
    mqtt.setup(MQTT_BROKER_URI, MQTT_CLIENT_ID, TOPIC_SUB);
    mqtt.publish(TOPIC_PUB, "ESP32 online");

    xTaskCreate(mqtt_task, "mqtt_task", 4096, NULL, 5, NULL);

    while (1)
    {
        if (Timer.interruptAvailable())
        {
            // Z homing: active every tick while z_homing is true
            if (z_homing)
            {
                if (Z_LimitSwitch.get() == 0)   // active-low: switch closed = home reached
                {
                    // Convert 190 mm → steps using same mapping as j2 readout:
                    //   j2 = steps * (360 / 1600) / Z_RATIO  →  steps = mm * 1600 * Z_RATIO / 360
                    int32_t home_steps = (int32_t)(Z_HOME_MM * 1600.0f * Z_RATIO / 360.0f);
                    Z_Stepper.forceStop();
                    Z_Stepper.resetPosition(home_steps);  // define this point as 190 mm
                    z_homing = false;
                    mqtt.publish(TOPIC_PUB, "Z:homed");
                    printf("Z homing complete — position set to %.0f mm\n", Z_HOME_MM);
                }
                else
                {
                    Z_Stepper.update();          // keep stepping toward large-positive target
                }
            }

            switch (mode)
            {
                // local variable before while(1)

            case 0:
                Base_Stepper.goToAngle(theta1 * BASE_RATIO);
                Z_Stepper.goToAngle(theta2 * Z_RATIO, 4000); // fixed base freq, no prev needed
                arm_motor.setTargetPosition(theta3 * ARM_RATIO);

                Z_Stepper.update();
                arm_motor.update();
                break;
            case 1:
            {
                // Get current joint positions from encoders (convert motor→joint)
                float current_j1 = Base_Stepper._encoder.getAccumulatedAngleDeg() / BASE_RATIO;
                float current_j2 = arm_motor.getPosition() / ARM_RATIO;
                float current_j3 = static_cast<float>(Z_Stepper.getPosition()) / Z_RATIO;
                float current_j4 = theta4;

                // Solve IK for target position
                IKSolution target_solution;
                bool reachable = robot.solveIK(cmd_x, cmd_y, cmd_z, current_j4,
                                                current_j1, current_j2, current_j3, current_j4,
                                                target_solution);

                if (reachable)
                {
                    // Convert to degrees for motor commands
                    theta1 = target_solution.theta1 * 57.29577951f;
                    theta3 = target_solution.theta2 * 57.29577951f;
                    theta2 = target_solution.z;
                    theta4 = target_solution.theta4 * 57.29577951f;

                    // Apply to motors (joint→motor)
                    Base_Stepper.goToAngle(theta1 * BASE_RATIO);
                    Z_Stepper.goToAngle(theta2 * Z_RATIO, 4000);
                    arm_motor.setTargetPosition(theta3 * ARM_RATIO);

                    Z_Stepper.update();
                    arm_motor.update();
                }
                else
                {
                    printf("IK unreachable for xyz:(%.2f,%.2f,%.2f)\n", cmd_x, cmd_y, cmd_z);
                }
                break;
            }

            case 2:  // trigger Z homing
                z_homing = true;
                // Positive = down; command a large positive target so the motor
                // moves DOWN continuously until the limit switch fires.
                Z_Stepper.goToAngle(99999.0f * Z_RATIO, 2000);
                mode = -1;   // consume the command
                break;

            default:
                break;
            }

            // Get current joint angles from encoders (convert motor→joint)
            float j1 = Base_Stepper._encoder.getAccumulatedAngleDeg() / BASE_RATIO;
            float j2 = (Z_Stepper.getPosition() * 360.0f / 1600.0f) / Z_RATIO;
            float j3 = arm_motor.getPosition() / ARM_RATIO;
            float j4 = theta4;

            // Publish at 10 Hz (every 10 ticks × 10 ms = 100 ms)
            if (++pub_tick >= 10)
            {
                pub_tick = 0;

                // Compute end-effector position via forward kinematics
                EndEffectorPose pose = robot.getEndEffectorPosition(j1, j3, j2, j4);

                snprintf(pub_buf, sizeof(pub_buf),
                         "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
                         j1, j2, j3, j4, pose.x, pose.y, pose.z);
                mqtt.publish(TOPIC_PUB, pub_buf);
            }
        }
    }
}
