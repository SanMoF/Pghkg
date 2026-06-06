#include "definitions.h"

static void control_task(void *pvParameters); // defined below app_main

static void IRAM_ATTR timerinterrupt(void *arg)
{
    Timer.setInterrupt();
}

// Enqueue a status string for the comms task to publish. Non-blocking: it
// copies the message and returns immediately (dropping it if the queue is
// full), so the real-time control loop never waits on the network.
static void queuePublish(const char *msg)
{
    if (mqtt_tx_q == nullptr)
        return;
    char buf[MQTT_MSG_LEN];
    strncpy(buf, msg, MQTT_MSG_LEN - 1);
    buf[MQTT_MSG_LEN - 1] = '\0';
    xQueueSend(mqtt_tx_q, buf, 0); // 0 ticks → never blocks
}

// Comms TX task — drains the queue and publishes. Runs off the control core,
// so a blocking esp_mqtt/TCP write can never stall motor control.
static void comms_tx_task(void *pvParameters)
{
    char buf[MQTT_MSG_LEN];
    while (1)
    {
        if (xQueueReceive(mqtt_tx_q, buf, portMAX_DELAY) == pdTRUE)
            mqtt.publish(TOPIC_PUB, buf);
    }
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
                    cmd_base_deg = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    cmd_z_mm = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    cmd_elbow_deg = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    cmd_wrist_deg = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    cmd_servo_pwm = atof(token); // ← nuevo
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
                token = strtok(nullptr, ",");
                if (token)
                    cmd_tool_deg = atof(token);
                token = strtok(nullptr, ",");
                if (token)
                    cmd_servo_pwm = atof(token); // ← nuevo
            }
            else if (mode == 4)
            {
                // Optional payload: a single pick→place pair, then optional
                // close/open gripper duties. With no payload, run the
                // hardcoded list. Always (re)start the sequence.
                float vals[6];
                int   n = 0;
                while (n < 6 && (token = strtok(nullptr, ",")) != nullptr)
                    vals[n++] = atof(token);

                if (n == 6)
                {
                    pick_pts[0][0]  = vals[0];
                    pick_pts[0][1]  = vals[1];
                    pick_pts[0][2]  = vals[2];
                    place_pts[0][0] = vals[3];
                    place_pts[0][1] = vals[4];
                    place_pts[0][2] = vals[5];
                    pp_run_count    = 1;

                    token = strtok(nullptr, ",");
                    if (token)
                        gripper_close_pwm = atof(token);
                    token = strtok(nullptr, ",");
                    if (token)
                        gripper_open_pwm = atof(token);
                }
                else
                {
                    pp_run_count = PP_COUNT; // no/partial coords → hardcoded list
                }

                pp_restart = true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Wrap an angle (deg) into [-180, 180) so the wrist takes the equivalent short
// rotation instead of unwinding a full turn. The IK wrist angle
// (theta4 = tool - theta1 - theta2) can fall well outside ±180.
static float wrapDeg180(float deg)
{
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f)
        deg += 360.0f;
    return deg - 180.0f;
}

// Read the four joint values back in joint-space units (deg, deg, mm, deg).
static void readJointState(float &base_deg, float &elbow_deg,
                           float &z_mm, float &wrist_deg)
{
    base_deg  = Base_Stepper._encoder.getAccumulatedAngleDeg() / BASE_RATIO;
    elbow_deg = arm_motor.getPosition() / ARM_RATIO;
    z_mm      = (Z_Stepper.getPosition() * 360.0f /
                 (float)Z_Stepper.stepsPerRev()) / Z_RATIO;
    wrist_deg = wrist_motor.getPosition() / WRIST_RATIO;
}

// Solve IK for a Cartesian target and latch the joint targets. Returns false
// (leaving the previous targets untouched) if the point is unreachable.
static bool solveCartesianTarget(float x, float y, float z, float tool_deg)
{
    float cur_base, cur_elbow, cur_z_mm, cur_wrist;
    readJointState(cur_base, cur_elbow, cur_z_mm, cur_wrist);

    IKSolution sol;
    bool reachable = robot.solveIK(x, y, z, tool_deg,
                                   cur_base, cur_elbow, cur_z_mm, cur_wrist, sol);
    if (reachable)
    {
        target_base_deg  = sol.theta1 * 57.29577951f;
        target_elbow_deg = sol.theta2 * 57.29577951f;
        target_z_mm      = sol.z;
        // Pick & place ignores gripper heading: hold the wrist where it is so
        // the part keeps its pickup orientation (the IK theta4 is discarded).
        target_wrist_deg = cur_wrist;
        motors_active    = true;
    }
    return reachable;
}

// A move sub-state is "done" when all joints settle, or when it has waited
// PP_SETTLE_TIMEOUT ticks — so a joint that can't reach its target (e.g. a
// wrist with no authority) advances the sequence instead of deadlocking it.
static bool atJointTarget();
static bool ppArrived()
{
    if (atJointTarget())
        return true;
    if (pp_settle >= PP_SETTLE_TIMEOUT)
        return true; // joint can't reach (e.g. wrist) — advance anyway
    return false;
}

// Solve a pick&place waypoint and latch the joint targets. Returns false
// (targets unchanged) when the point is unreachable; the caller then holds the
// previous waypoint until the settle timeout advances the sequence.
static bool ppMoveTo(float x, float y, float z)
{
    return solveCartesianTarget(x, y, z, pp_tool_deg);
}

// True once base, elbow and Z are within tolerance of their latched targets.
// The wrist is intentionally excluded — pick & place doesn't control gripper
// heading, so it must not gate arrival.
static bool atJointTarget()
{
    float base_deg, elbow_deg, z_mm, wrist_deg;
    readJointState(base_deg, elbow_deg, z_mm, wrist_deg);

    return fabsf(base_deg  - target_base_deg)  < PP_ANG_TOL &&
           fabsf(elbow_deg - target_elbow_deg) < PP_ANG_TOL &&
           fabsf(z_mm      - target_z_mm)      < PP_Z_TOL;
}

extern "C" void app_main()
{
    esp_task_wdt_deinit();

    robot.setup(scara_l1, scara_l2, ARM_PLANE_HOME, TCP_Z_DROP);

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

    mqtt_tx_q = xQueueCreate(MQTT_TX_QUEUE_LEN, MQTT_MSG_LEN);

    // Networking (RX parse + TX publish) lives on core 0 with the WiFi stack.
    // The real-time motor loop gets core 1 to itself. Its priority (20) is set
    // ABOVE the lwIP/TCP-IP task (CONFIG_LWIP_TCPIP_TASK_PRIO = 18, which has
    // NO_AFFINITY and could otherwise land on core 1) so network activity can
    // never preempt stepper timing — the source of the intermittent Z jitter.
    xTaskCreatePinnedToCore(comms_tx_task, "comms_tx", 4096, NULL, 5,  NULL, 0);
    xTaskCreatePinnedToCore(mqtt_task,     "mqtt_rx",  4096, NULL, 5,  NULL, 0);
    xTaskCreatePinnedToCore(control_task,  "control",  8192, NULL, 20, NULL, 1);
}

// ── Real-time motor control — pinned to core 1, isolated from networking ─────
static void control_task(void *pvParameters)
{
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
                    target_z_mm = 0.0f; // hold at home
                    z_homing = false;
                    queuePublish("Z:homed");
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
                target_base_deg = cmd_base_deg;
                target_z_mm = cmd_z_mm;
                target_elbow_deg = cmd_elbow_deg;
                target_wrist_deg = cmd_wrist_deg;
                
                motors_active = true;
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
                    float cur_base = Base_Stepper._encoder.getAccumulatedAngleDeg() / BASE_RATIO;
                    float cur_elbow = arm_motor.getPosition() / ARM_RATIO;
                    float cur_z_mm = (Z_Stepper.getPosition() * 360.0f /
                                      (float)Z_Stepper.stepsPerRev()) /
                                     Z_RATIO;
                    float cur_wrist = wrist_motor.getPosition() / WRIST_RATIO;

                    IKSolution sol;
                    bool reachable = robot.solveIK(cmd_x, cmd_y, cmd_z, cmd_tool_deg,
                                                   cur_base, cur_elbow, cur_z_mm, cur_wrist,
                                                   sol);
                    if (reachable)
                    {
                        target_base_deg = sol.theta1 * 57.29577951f;
                        target_elbow_deg = sol.theta2 * 57.29577951f;
                        target_z_mm = sol.z;
                        target_wrist_deg = wrapDeg180(sol.theta4 * 57.29577951f);

                        last_cmd_x = cmd_x;
                        last_cmd_y = cmd_y;
                        last_cmd_z = cmd_z;
                        last_cmd_tool = cmd_tool_deg;
                        motors_active = true;
                    }
                    else
                    {
                        snprintf(pub_buf, sizeof(pub_buf),
                                 "IK:unreachable %.1f,%.1f,%.1f", cmd_x, cmd_y, cmd_z);
                        queuePublish(pub_buf);
                    }
                }
                break;
            }

            case 2: // trigger Z homing
                z_homing = true;
                Z_Stepper.goToAngle(-99999.0f * Z_RATIO, 2000);
                mode = -1;
                break;

            case 3: // declare current pose as zero for base/elbow/wrist
                Base_Stepper._encoder.resetAccumulatedAngle();
                arm_motor.zeroPosition();
                wrist_motor.zeroPosition();
                target_base_deg = 0.0f;
                target_elbow_deg = 0.0f;
                target_wrist_deg = 0.0f;
                last_cmd_x = last_cmd_y = last_cmd_z = last_cmd_tool = NAN;
                motors_active = true;
                queuePublish("JOINTS:zeroed");
                mode = -1;
                break;

            case 4: // pick & place sequence over the hardcoded waypoints
            {
                // Entering the mode (or a fresh mode-4 command): restart the
                // sequence with the gripper open.
                if (prev_mode != 4 || pp_restart)
                {
                    pp_index      = 0;
                    pp_state      = PP_APPROACH_PICK;
                    pp_new_state  = true;
                    pp_dwell      = 0;
                    pp_restart    = false;
                    cmd_servo_pwm = gripper_open_pwm;
                }

                // Finished all pairs — hold position.
                if (pp_index >= pp_run_count)
                {
                    pp_state = PP_DONE;
                    break;
                }

                const float px = pick_pts[pp_index][0];
                const float py = pick_pts[pp_index][1];
                const float pz = pick_pts[pp_index][2];
                const float qx = place_pts[pp_index][0];
                const float qy = place_pts[pp_index][1];
                const float qz = place_pts[pp_index][2];

                int pp_state_before = pp_state;

                switch (pp_state)
                {
                case PP_APPROACH_PICK:
                    if (pp_new_state)
                    {
                        pp_reachable = ppMoveTo(px, py, pz + PP_APPROACH_MM);
                        pp_new_state = false;
                    }
                    if (pp_reachable && ppArrived())
                    {
                        pp_state = PP_DESCEND_PICK;
                        pp_new_state = true;
                    }
                    break;

                case PP_DESCEND_PICK:
                    if (pp_new_state)
                    {
                        pp_reachable = ppMoveTo(px, py, pz);
                        pp_new_state = false;
                    }
                    if (pp_reachable && ppArrived())
                    {
                        pp_state = PP_GRIP_CLOSE;
                        pp_new_state = true;
                    }
                    break;

                case PP_GRIP_CLOSE:
                    if (pp_new_state)
                    {
                        cmd_servo_pwm = gripper_close_pwm;
                        pp_dwell = 0;
                        pp_new_state = false;
                    }
                    if (++pp_dwell >= PP_GRIP_TICKS)
                    {
                        pp_state = PP_LIFT_PICK;
                        pp_new_state = true;
                    }
                    break;

                case PP_LIFT_PICK:
                    if (pp_new_state)
                    {
                        pp_reachable = ppMoveTo(px, py, pz + PP_APPROACH_MM);
                        pp_new_state = false;
                    }
                    if (pp_reachable && ppArrived())
                    {
                        pp_state = PP_APPROACH_PLACE;
                        pp_new_state = true;
                    }
                    break;

                case PP_APPROACH_PLACE:
                    if (pp_new_state)
                    {
                        pp_reachable = ppMoveTo(qx, qy, qz + PP_APPROACH_MM);
                        pp_new_state = false;
                    }
                    if (pp_reachable && ppArrived())
                    {
                        pp_state = PP_DESCEND_PLACE;
                        pp_new_state = true;
                    }
                    break;

                case PP_DESCEND_PLACE:
                    if (pp_new_state)
                    {
                        pp_reachable = ppMoveTo(qx, qy, qz);
                        pp_new_state = false;
                    }
                    if (pp_reachable && ppArrived())
                    {
                        pp_state = PP_GRIP_OPEN;
                        pp_new_state = true;
                    }
                    break;

                case PP_GRIP_OPEN:
                    if (pp_new_state)
                    {
                        cmd_servo_pwm = gripper_open_pwm;
                        pp_dwell = 0;
                        pp_new_state = false;
                    }
                    if (++pp_dwell >= PP_GRIP_TICKS)
                    {
                        pp_state = PP_LIFT_PLACE;
                        pp_new_state = true;
                    }
                    break;

                case PP_LIFT_PLACE:
                    if (pp_new_state)
                    {
                        pp_reachable = ppMoveTo(qx, qy, qz + PP_APPROACH_MM);
                        pp_new_state = false;
                    }
                    if (pp_reachable && ppArrived())
                    {
                        pp_index++; // next pick/place pair
                        pp_state = PP_APPROACH_PICK;
                        pp_new_state = true;
                    }
                    break;

                default:
                    break;
                }

                // Maintain the settle timer (reset on each sub-state change).
                // No serial logging in the loop — console I/O is blocking and
                // stalled the 10 ms control loop, starving the steppers. The
                // live state is reported via the 10 Hz MQTT telemetry instead.
                if (pp_state != pp_state_before)
                    pp_settle = 0;
                else
                    pp_settle++;
                break;
            }

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
                    Servo.setDuty(cmd_servo_pwm);   // ← nuevo, ajusta el nombre al método real de tu clase

                if (!z_homing) // homing block already drives Z
                {
                    Z_Stepper.goToAngle(target_z_mm * Z_RATIO, 4000);
                    Z_Stepper.update();
                }
            }

            // ── Telemetry at 10 Hz ─────────────────────────────────
            if (++pub_tick >= 10)
            {
                pub_tick = 0;

                float base_deg = Base_Stepper._encoder.getAccumulatedAngleDeg() / BASE_RATIO;
                float z_mm = (Z_Stepper.getPosition() * 360.0f /
                              (float)Z_Stepper.stepsPerRev()) /
                             Z_RATIO;
                float elbow_deg = arm_motor.getPosition() / ARM_RATIO;
                float wrist_deg = wrist_motor.getPosition() / WRIST_RATIO;

                EndEffectorPose pose = robot.getEndEffectorPosition(
                    base_deg, elbow_deg, z_mm, wrist_deg);

                // Actual joint speeds: differentiate the joint positions over
                // the telemetry interval (pub_tick wraps every 10 control
                // ticks → 10 * dt µs). First pass reports 0 to avoid a spike.
                float tele_dt = 10.0f * dt * 1e-6f; // s
                float base_dps  = 0.0f, z_mmps = 0.0f;
                float elbow_dps = 0.0f, wrist_dps = 0.0f;
                if (have_prev_joint)
                {
                    base_dps  = (base_deg  - prev_base_deg ) / tele_dt;
                    z_mmps    = (z_mm      - prev_z_mm     ) / tele_dt;
                    elbow_dps = (elbow_deg - prev_elbow_deg) / tele_dt;
                    wrist_dps = (wrist_deg - prev_wrist_deg) / tele_dt;
                }
                prev_base_deg   = base_deg;
                prev_z_mm       = z_mm;
                prev_elbow_deg  = elbow_deg;
                prev_wrist_deg  = wrist_deg;
                have_prev_joint = true;

                // Operational velocity of the TCP via the SCARA Jacobian.
                EndEffectorTwist twist = robot.getEndEffectorVelocity(
                    base_deg, elbow_deg, base_dps, elbow_dps, z_mmps, wrist_dps);

                // Telemetry: joint positions, TCP pose, joint speeds, TCP
                // velocity, and the gripper state (current servo duty).
                snprintf(pub_buf, sizeof(pub_buf),
                         "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                         "%.2f,%.2f,%.2f,%.2f,"
                         "%.2f,%.2f,%.2f,%.2f,"
                         "%.2f",
                         base_deg, z_mm, elbow_deg, wrist_deg,
                         pose.x, pose.y, pose.z,
                         base_dps, z_mmps, elbow_dps, wrist_dps,
                         twist.vx, twist.vy, twist.vz, twist.omega,
                         cmd_servo_pwm);
                queuePublish(pub_buf);
            }
        }
    }
}
