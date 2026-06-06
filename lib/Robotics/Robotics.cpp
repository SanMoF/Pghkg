#include "Robotics.h"
#include <math.h>

Robotics::Robotics(/* args */)
{
}

Robotics::~Robotics()
{
}

void Robotics::setup(float link1_length, float link2_length,
                     float z_arm_plane_home, float tcp_z_drop)
{
    L1 = link1_length;
    L2 = link2_length;
    Z_arm_plane_home = z_arm_plane_home;
    tcp_drop         = tcp_z_drop;
}

EndEffectorPose Robotics::getEndEffectorPosition(float theta1_deg, float theta2_deg,
                                                  float z_travel, float theta4_deg)
{
    
    float dh[4][4] = {
        {theta1_deg * DEG_TO_RAD, 0.0f, L1, 0.0f},
        {theta2_deg * DEG_TO_RAD, 0.0f, L2, 0.0f},
        {0.0f, Z_arm_plane_home - z_travel, 0.0f, 0.0f},
        {theta4_deg * DEG_TO_RAD, -tcp_drop, 0.0f, 0.0f}};

    float tf[16] = {};
    Forward_Kinematics(dh, 4, tf);

    EndEffectorPose pose;
    pose.x = tf[3];
    pose.y = tf[7];
    pose.z = tf[11];
    return pose;
}

EndEffectorTwist Robotics::getEndEffectorVelocity(float theta1_deg, float theta2_deg,
                                                  float dtheta1_dps, float dtheta2_dps,
                                                  float dz_travel_mmps, float dtheta4_dps)
{
    float t1 = theta1_deg * DEG_TO_RAD;
    float t2 = theta2_deg * DEG_TO_RAD;

    float w1 = dtheta1_dps * DEG_TO_RAD;
    float w2 = dtheta2_dps * DEG_TO_RAD;

    float s1  = sinf(t1);
    float c1  = cosf(t1);
    float s12 = sinf(t1 + t2);
    float c12 = cosf(t1 + t2);


    float J11 = -L1 * s1 - L2 * s12;
    float J12 = -L2 * s12;
    float J21 =  L1 * c1 + L2 * c12;
    float J22 =  L2 * c12;

    EndEffectorTwist tw;
    tw.vx = J11 * w1 + J12 * w2;                 // mm/s
    tw.vy = J21 * w1 + J22 * w2;                 // mm/s
    tw.vz = -dz_travel_mmps;                     // mm/s, +up
    tw.omega = dtheta1_dps + dtheta2_dps + dtheta4_dps; // deg/s
    return tw;
}

bool Robotics::solveIK(float target_x, float target_y, float target_z, float target_tool_angle_deg,
                       float current_theta1_deg, float current_theta2_deg,
                       float current_z, float current_theta4_deg,
                       IKSolution &best_solution)
{
    IKSolution solutions[2] = {};
    float tool_angle_rad = target_tool_angle_deg * DEG_TO_RAD;

    int num_solutions = Inverse_Kinematics(target_x, target_y, target_z, tool_angle_rad, solutions);

    if (num_solutions == 0)
        return false;

    // Build current joints array (in radians for findBestSolution)
    float current_joints[4] = {
        current_theta1_deg * DEG_TO_RAD,
        current_theta2_deg * DEG_TO_RAD,
        current_z,
        current_theta4_deg * DEG_TO_RAD};

    float weights[3] = {1.0f, 1.0f, 0.2f};
    int best_idx = findBestSolution(solutions, num_solutions, current_joints, weights);

    best_solution = solutions[best_idx];
    return true;
}



int Robotics::Inverse_Kinematics(float x, float y, float z, float tool_angle, IKSolution solutions[2])
{
    float p = sqrtf(x * x + y * y);
    float max_reach = L1 + L2;
    float min_reach = fabsf(L1 - L2);

    if (p > max_reach + 1e-4f || p < min_reach - 1e-4f)
        return 0;

    p = fmaxf(min_reach, fminf(max_reach, p));

    float gamma = atan2f(y, x);
    float cos_alpha = (L1 * L1 + p * p - L2 * L2) / (2.0f * L1 * p);
    float cos_beta  = (L2 * L2 + L1 * L1 - p * p) / (2.0f * L1 * L2);

    cos_alpha = fmaxf(-1.0f, fminf(1.0f, cos_alpha));
    cos_beta  = fmaxf(-1.0f, fminf(1.0f, cos_beta));

    float alpha = acosf(cos_alpha);
    float beta  = acosf(cos_beta);

    float z_travel = getHomeTcpHeight() - z;
    if (z_travel < 0.0f)
        z_travel = 0.0f;

    solutions[0].theta1 = gamma - alpha;
    solutions[0].theta2 = M_PI - beta;
    solutions[0].z      = z_travel;
    solutions[0].theta4 = tool_angle - solutions[0].theta1 - solutions[0].theta2;

    if (fabsf(p - max_reach) < 1e-4f || fabsf(p - min_reach) < 1e-4f)
        return 1;

    solutions[1].theta1 = gamma + alpha;
    solutions[1].theta2 = beta - M_PI;
    solutions[1].z      = z_travel;
    solutions[1].theta4 = tool_angle - solutions[1].theta1 - solutions[1].theta2;

    return 2;
}
int Robotics::findBestSolution(IKSolution solutions[2], int num_solutions,
                               float current_joints[4], float weights[3])
{
    if (num_solutions == 1)
        return 0;

    float best_error = 1e9f;
    int best_index = 0;

    float sol_joints[2][3] = {
        {solutions[0].theta1, solutions[0].theta2, solutions[0].theta4},
        {solutions[1].theta1, solutions[1].theta2, solutions[1].theta4}};
    float current[3] = {current_joints[0], current_joints[1], current_joints[3]};

    for (int i = 0; i < num_solutions; i++)
    {
        float error = 0.0f;
        for (int j = 0; j < 3; j++)
            error += weights[j] * fabsf(sol_joints[i][j] - current[j]);

        if (error < best_error)
        {
            best_error = error;
            best_index = i;
        }
    }

    return best_index;
}

void Robotics::Matrix_multiplication(float *M1, float *M2, float *Result, int rows1, int cols1, int cols2)
{
    for (int i = 0; i < rows1 * cols2; i++)
        Result[i] = 0;

    for (int i = 0; i < rows1; i++)
    {
        for (int j = 0; j < cols2; j++)
        {
            for (int k = 0; k < cols1; k++)
            {
                Result[i * cols2 + j] += M1[i * cols1 + k] * M2[k * cols2 + j];
            }
        }
    }
}

void Robotics::DH_to_HMatrix(float DH_Parameters[4], float *Result)
{
    float theta = DH_Parameters[0];
    float d = DH_Parameters[1];
    float a = DH_Parameters[2];
    float alpha = DH_Parameters[3];

    float ct = cos(theta);
    float st = sin(theta);
    float ca = cos(alpha);
    float sa = sin(alpha);

    // Fila 0
    Result[0] = ct;
    Result[1] = -st * ca;
    Result[2] = st * sa;
    Result[3] = a * ct;
    // Fila 1
    Result[4] = st;
    Result[5] = ct * ca;
    Result[6] = -ct * sa;
    Result[7] = a * st;
    // Fila 2
    Result[8] = 0;
    Result[9] = sa;
    Result[10] = ca;
    Result[11] = d;
    // Fila 3
    Result[12] = 0;
    Result[13] = 0;
    Result[14] = 0;
    Result[15] = 1;
}

void Robotics::Forward_Kinematics(float DH_table[][4], uint8_t num_joints, float *Result)
{
    float identity[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    for (int i = 0; i < 16; i++)
        Result[i] = identity[i];

    for (int joint = 0; joint < num_joints; joint++)
    {
        float Ti[16] = {};
        float Temp[16] = {};
        DH_to_HMatrix(DH_table[joint], Ti);
        Matrix_multiplication(Result, Ti, Temp, 4, 4, 4);
        for (int i = 0; i < 16; i++)
            Result[i] = Temp[i];
    }
}