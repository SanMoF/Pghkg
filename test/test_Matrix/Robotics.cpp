
#include "Robotics.h"
#include <math.h>


Robotics::Robotics(/* args */)
{

}


Robotics::~Robotics()
{

}


void Robotics::setup(float link1_length, float link2_length)
{
    L1 = link1_length;
    L2 = link2_length;
}

void Robotics::setLinkLengths(float link1, float link2)
{
    L1 = link1;
    L2 = link2;
}

void Robotics::Inverse_Kinematics(float x, float y, float z, float *theta1, float *theta2, float *theta3, bool elbow_up)
{
    // SCARA inverse kinematics
    // Joint 1 (base rotation around Z)
    *theta1 = atan2(y, x);  // Rotation around Z axis to point towards target

    // Distance in XY plane from base to target
    float r = sqrt(x * x + y * y);

    // Law of cosines for theta2 (elbow angle between links)
    float cos_theta2 = (r * r - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
    cos_theta2 = fmax(-1.0f, fmin(1.0f, cos_theta2));  // Clamp for safety

    if (elbow_up) {
        *theta2 = acos(cos_theta2);  // Elbow up configuration
    } else {
        *theta2 = -acos(cos_theta2); // Elbow down configuration
    }

    // Theta3 is the wrist rotation (Z axis rotation of end effector)
    // For a standard SCARA, theta3 compensates to keep end effector level
    // Here we set it as the remaining angle to complete the orientation
    float psi = atan2(y, x);
    float phi = atan2(L2 * sin(*theta2), L1 + L2 * cos(*theta2));
    *theta3 = psi - phi;

    // Convert to degrees for joint space (if needed)
    // *theta1 *= 180.0f / M_PI;
    // *theta2 *= 180.0f / M_PI;
    // *theta3 *= 180.0f / M_PI;
}


void Robotics::Matrix_multiplication(float* M1, float* M2, float* Result, int rows1, int cols1, int cols2)
{
    for (int i = 0; i < rows1 * cols2; i++)
        Result[i] = 0;

    for (int i = 0; i < rows1; i++) {
        for (int j = 0; j < cols2; j++) {
            for (int k = 0; k < cols1; k++) {
                Result[i * cols2 + j] += M1[i * cols1 + k] * M2[k * cols2 + j];
            }
        }
    }
}

void Robotics::DH_to_HMatrix(float DH_Parameters[4], float* Result)
{
    float theta =DH_Parameters[0]; 
    float d =  DH_Parameters[1];
    float a = DH_Parameters[2];
    float alpha = DH_Parameters[3];
    
    float ct = cos(theta);
    float st = sin(theta);
    float ca = cos(alpha);
    float sa = sin(alpha);

    // Fila 0
    Result[0]  =  ct;    Result[1]  = -st*ca;  Result[2]  =  st*sa;  Result[3]  = a*ct;
    // Fila 1
    Result[4]  =  st;    Result[5]  =  ct*ca;  Result[6]  = -ct*sa;  Result[7]  = a*st;
    // Fila 2
    Result[8]  =  0;     Result[9]  =  sa;     Result[10] =  ca;     Result[11] = d;
    // Fila 3
    Result[12] =  0;     Result[13] =  0;      Result[14] =  0;      Result[15] = 1;
}

void Robotics::Forward_Kinematics(float DH_table[][4], uint8_t num_joints, float *Result)
{
    float identity[16] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    for (int i = 0; i < 16; i++) Result[i] = identity[i];

    for (int joint = 0; joint < num_joints; joint++) {
        float Ti[16] = {};
        float Temp[16] = {};
        DH_to_HMatrix(DH_table[joint], Ti);
        Matrix_multiplication(Result, Ti, Temp, 4, 4, 4);
        for (int i = 0; i < 16; i++) Result[i] = Temp[i];
    }
}
