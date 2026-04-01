#ifndef __ROBOTICS_H__
#define __ROBOTICS_H__

#include <cstdint>

struct IKSolution
{
    float theta1; // degrees
    float theta2; // degrees
    float z;      // linear, same units as link lengths
    float theta4; // degrees
};

class Robotics
{
private:
    float L1 = 100.0f; // Link 1 length (mm)
    float L2 = 100.0f; // Link 2 length (mm)

public:
    Robotics();
    ~Robotics();

    void setup(float link1_length, float link2_length);
    void setLinkLengths(float link1, float link2);
    float getLink1Length() const { return L1; }
    float getLink2Length() const { return L2; }

    void Matrix_multiplication(float *M1, float *M2, float *Result, int rows1, int cols1, int cols2);
    void DH_to_HMatrix(float DH_Parameters[4], float *Result);
    void Forward_Kinematics(float DH_table[][4], uint8_t num_joints, float *Result);
    int Inverse_Kinematics(float x, float y, float z, float tool_angle, IKSolution solutions[2]);
    int findBestSolution(IKSolution solutions[2], int num_solutions,
                         float current_joints[4], float weights[3]);
};

#endif // __ROBOTICS_H__