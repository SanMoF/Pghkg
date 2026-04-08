#ifndef __ROBOTICS_H__
#define __ROBOTICS_H__

#include <cstdint>

struct IKSolution
{
    float theta1; // radians
    float theta2; // radians
    float z;      // linear, same units as link lengths
    float theta4; // radians
};

struct EndEffectorPose
{
    float x;
    float y;
    float z;
};

class Robotics
{
private:
    float L1 = 100.0f; // Link 1 length (mm)
    float L2 = 100.0f; // Link 2 length (mm)
    float dh_table[4][4]; // Internal DH parameter storage

    static constexpr float DEG_TO_RAD = 0.01745329252f;
    static constexpr float RAD_TO_DEG = 57.29577951f;

public:
    Robotics();
    ~Robotics();

    void setup(float link1_length, float link2_length);
    void setLinkLengths(float link1, float link2);
    float getLink1Length() const { return L1; }
    float getLink2Length() const { return L2; }

    // Forward kinematics from joint angles (degrees) -> end-effector position
    EndEffectorPose getEndEffectorPosition(float theta1_deg, float theta2_deg,
                                           float z_pos, float theta4_deg);

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