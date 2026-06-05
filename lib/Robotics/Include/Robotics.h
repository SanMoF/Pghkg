#ifndef __ROBOTICS_H__
#define __ROBOTICS_H__

#include <cstdint>

struct IKSolution
{
    float theta1; // radians
    float theta2; // radians
    float z;      // leadscrew travel (mm) from the homed top position, +down
    float theta4; // radians
};

struct EndEffectorPose
{
    float x;
    float y;
    float z;
};

// Operational-space velocity of the TCP (the end-effector "twist").
struct EndEffectorTwist
{
    float vx;    // mm/s  (along base X)
    float vy;    // mm/s  (along base Y)
    float vz;    // mm/s  (+up, world Z)
    float omega; // deg/s (TCP yaw rate about Z)
};

class Robotics
{
private:
    float L1 = 100.0f; // Link 1 length (mm)
    float L2 = 100.0f; // Link 2 length (mm)
    float dh_table[4][4]; // Internal DH parameter storage

    // ── Vertical geometry (Z chain) ───────────────────────────────────
    // The two rotary links sweep a horizontal "arm plane". A leadscrew
    // raises/lowers the tool; its travel is measured 0 at the homed TOP
    // position and grows positive as the tool moves DOWN. The TCP hangs a
    // fixed distance below the arm plane (the tool drop).
    //
    //   TCP_height = (arm_plane_home - travel) - tcp_drop
    //              = home_tcp_height - travel
    //
    // With arm_plane_home = 340 mm and tcp_drop = 130 mm the homed TCP sits
    // at 210 mm, matching the measured home height without a tool.
    float Z_arm_plane_home = 340.0f; // arm-plane height (mm) when Z is homed
    float tcp_drop         = 130.0f; // fixed vertical wrist->TCP drop (mm)

    static constexpr float DEG_TO_RAD = 0.01745329252f;
    static constexpr float RAD_TO_DEG = 57.29577951f;

public:
    Robotics();
    ~Robotics();

    void setup(float link1_length, float link2_length,
               float z_arm_plane_home = 340.0f, float tcp_z_drop = 130.0f);
    void setLinkLengths(float link1, float link2);
    void setZGeometry(float z_arm_plane_home, float tcp_z_drop)
    {
        Z_arm_plane_home = z_arm_plane_home;
        tcp_drop         = tcp_z_drop;
    }
    float getLink1Length() const { return L1; }
    float getLink2Length() const { return L2; }
    float getHomeTcpHeight() const { return Z_arm_plane_home - tcp_drop; }

    // Forward kinematics from joint state -> end-effector pose.
    // z_travel is leadscrew travel (mm) from the homed top position (+down);
    // the returned pose.z is the absolute TCP height above the base.
    EndEffectorPose getEndEffectorPosition(float theta1_deg, float theta2_deg,
                                           float z_travel, float theta4_deg);

    // Operational (Cartesian) velocity from the joint velocities via the
    // SCARA Jacobian. Angles are the current joint *positions* (deg) and the
    // d*_dt arguments are the current joint *speeds*:
    //   dtheta1_dps / dtheta2_dps / dtheta4_dps : deg/s  (base, elbow, wrist)
    //   dz_travel_mmps                          : mm/s   (leadscrew travel, +down)
    // Returns the TCP twist (vx, vy, vz in mm/s, omega in deg/s).
    EndEffectorTwist getEndEffectorVelocity(float theta1_deg, float theta2_deg,
                                            float dtheta1_dps, float dtheta2_dps,
                                            float dz_travel_mmps, float dtheta4_dps);

    // Solve IK and return best solution for given target and current state
    // Returns true if solution found, false if unreachable
    bool solveIK(float target_x, float target_y, float target_z, float target_tool_angle_deg,
                 float current_theta1_deg, float current_theta2_deg,
                 float current_z, float current_theta4_deg,
                 IKSolution &best_solution);

    // Low-level methods (still available if needed)
    void Matrix_multiplication(float *M1, float *M2, float *Result, int rows1, int cols1, int cols2);
    void DH_to_HMatrix(float DH_Parameters[4], float *Result);
    void Forward_Kinematics(float DH_table[][4], uint8_t num_joints, float *Result);
    int Inverse_Kinematics(float x, float y, float z, float tool_angle, IKSolution solutions[2]);
    int findBestSolution(IKSolution solutions[2], int num_solutions,
                         float current_joints[4], float weights[3]);
};

#endif // __ROBOTICS_H__