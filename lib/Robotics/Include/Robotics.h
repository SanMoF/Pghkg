#ifndef __ROBOTICS_H__
#define __ROBOTICS_H__
#include "SimpleGPIO.h"

class Robotics
{
private:
    float L1 = 100.0f;  // Link 1 length (mm)
    float L2 = 100.0f;  // Link 2 length (mm)
public:
    Robotics(/* args */);
    ~Robotics();
    void setup(float link1_length, float link2_length);
    void Matrix_multiplication(float *M1, float *M2, float *Result, int rows1, int cols1, int cols2);
    void DH_to_HMatrix(float DH_Parameters[4], float *Result);
    void Forward_Kinematics(float DH_table[][4], uint8_t num_joints, float *Result);
    void Inverse_Kinematics(float x, float y, float z, float *theta1, float *theta2, float *theta3, bool elbow_up = false);
    void setLinkLengths(float link1, float link2);
    float getLink1Length() const { return L1; }
    float getLink2Length() const { return L2; }
};

#endif // __ROBOTICS_H__