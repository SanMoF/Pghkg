#include <unity.h>
#include <math.h>
#include <cstdio>
#include "Robotics.h"

Robotics robot;

// ═════════════════════════════════════════════════════════════════════════
// HELPER FUNCTIONS
// ═════════════════════════════════════════════════════════════════════════

static bool approx_equal(float* A, float* B, int n, float tol = 1e-4f) {
    for (int i = 0; i < n; i++) {
        if (fabs(A[i] - B[i]) > tol) {
            return false;
        }
    }
    return true;
}

void print_matrix_4x4(float* M, const char* name) {
    printf("\n%s:\n", name);
    for (int i = 0; i < 4; i++) {
        printf("  [");
        for (int j = 0; j < 4; j++) {
            printf("%8.4f ", M[i*4 + j]);
        }
        printf("]\n");
    }
}

// ═════════════════════════════════════════════════════════════════════════
// MATRIX MULTIPLICATION TESTS
// ═════════════════════════════════════════════════════════════════════════

void setUp(void) {
    // Reset robot configuration before each test
    robot.setLinkLengths(0.3f, 0.3f);
}

void tearDown(void) {
}

void test_matmul_identity_times_identity(void) {
    float I[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    float R[4][4] = {};
    robot.Matrix_multiplication(&I[0][0], &I[0][0], &R[0][0], 4, 4, 4);
    float expected[] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    TEST_ASSERT_TRUE(approx_equal(&R[0][0], expected, 16));
}

void test_matmul_A_times_identity_equals_A(void) {
    float A[4][4] = {{1,2,3,4},{5,6,7,8},{9,10,11,12},{13,14,15,16}};
    float I[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    float R[4][4] = {};
    robot.Matrix_multiplication(&A[0][0], &I[0][0], &R[0][0], 4, 4, 4);
    float expected[] = {1,2,3,4, 5,6,7,8, 9,10,11,12, 13,14,15,16};
    TEST_ASSERT_TRUE(approx_equal(&R[0][0], expected, 16));
}

void test_matmul_2x2_known_result(void) {
    float A[2][2] = {{1,2},{3,4}};
    float B[2][2] = {{5,6},{7,8}};
    float R[2][2] = {};
    robot.Matrix_multiplication(&A[0][0], &B[0][0], &R[0][0], 2, 2, 2);
    float expected[] = {19,22, 43,50};
    TEST_ASSERT_TRUE(approx_equal(&R[0][0], expected, 4));
}

void test_matmul_zero_matrix(void) {
    float A[4][4] = {{1,2,3,4},{5,6,7,8},{9,10,11,12},{13,14,15,16}};
    float Z[4][4] = {};
    float R[4][4] = {};
    robot.Matrix_multiplication(&A[0][0], &Z[0][0], &R[0][0], 4, 4, 4);
    float expected[16] = {};
    TEST_ASSERT_TRUE(approx_equal(&R[0][0], expected, 16));
}

void test_matmul_homogeneous_translation(void) {
    float T1[4][4] = {{1,0,0,1},{0,1,0,2},{0,0,1,3},{0,0,0,1}};
    float T2[4][4] = {{1,0,0,4},{0,1,0,5},{0,0,1,6},{0,0,0,1}};
    float R[4][4]  = {};
    robot.Matrix_multiplication(&T1[0][0], &T2[0][0], &R[0][0], 4, 4, 4);
    float expected[] = {1,0,0,5, 0,1,0,7, 0,0,1,9, 0,0,0,1};
    TEST_ASSERT_TRUE(approx_equal(&R[0][0], expected, 16));
}

void test_matmul_non_square_2x3_times_3x2(void) {
    float A[2][3] = {{1,2,3},{4,5,6}};
    float B[3][2] = {{7,8},{9,10},{11,12}};
    float R[2][2] = {};
    robot.Matrix_multiplication(&A[0][0], &B[0][0], &R[0][0], 2, 3, 2);
    // [1*7+2*9+3*11, 1*8+2*10+3*12] = [58, 64]
    // [4*7+5*9+6*11, 4*8+5*10+6*12] = [139,154]
    float expected[] = {58,64, 139,154};
    TEST_ASSERT_TRUE(approx_equal(&R[0][0], expected, 4));
}

// ═════════════════════════════════════════════════════════════════════════
// DH TO HOMOGENEOUS MATRIX TESTS
// ═════════════════════════════════════════════════════════════════════════

void test_DH_all_zeros_gives_identity(void) {
    float params[4] = {0, 0, 0, 0};
    float R[4][4] = {};
    robot.DH_to_HMatrix(params, &R[0][0]);
    float expected[] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    TEST_ASSERT_TRUE(approx_equal(&R[0][0], expected, 16));
}

void test_DH_rotation_z_90(void) {
    float params[4] = {(float)M_PI/2, 0, 0, 0};
    float R[4][4] = {};
    robot.DH_to_HMatrix(params, &R[0][0]);
    float expected[] = {
         0, -1, 0, 0,
         1,  0, 0, 0,
         0,  0, 1, 0,
         0,  0, 0, 1
    };
    TEST_ASSERT_TRUE(approx_equal(&R[0][0], expected, 16));
}

void test_DH_translation_z(void) {
    float params[4] = {0, 5, 0, 0};
    float R[4][4] = {};
    robot.DH_to_HMatrix(params, &R[0][0]);
    float expected[] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 5,
        0, 0, 0, 1
    };
    TEST_ASSERT_TRUE(approx_equal(&R[0][0], expected, 16));
}

void test_DH_translation_x(void) {
    float params[4] = {0, 0, 3, 0};
    float R[4][4] = {};
    robot.DH_to_HMatrix(params, &R[0][0]);
    float expected[] = {
        1, 0, 0, 3,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    };
    TEST_ASSERT_TRUE(approx_equal(&R[0][0], expected, 16));
}

void test_DH_rotation_x_90(void) {
    float params[4] = {0, 0, 0, (float)M_PI/2};
    float R[4][4] = {};
    robot.DH_to_HMatrix(params, &R[0][0]);
    float expected[] = {
        1,  0,  0, 0,
        0,  0, -1, 0,
        0,  1,  0, 0,
        0,  0,  0, 1
    };
    TEST_ASSERT_TRUE(approx_equal(&R[0][0], expected, 16));
}

void test_DH_last_row_always_0001(void) {
    float params[4] = {1.2f, 3.4f, 5.6f, 7.8f};
    float R[4][4] = {};
    robot.DH_to_HMatrix(params, &R[0][0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, R[3][0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, R[3][1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, R[3][2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1, R[3][3]);
}

// ═════════════════════════════════════════════════════════════════════════
// FORWARD KINEMATICS TESTS
// ═════════════════════════════════════════════════════════════════════════

void test_FK_zero_joints_gives_identity(void) {
    float DH[1][4] = {{0,0,0,0}};
    float R[4][4] = {};
    robot.Forward_Kinematics(DH, 0, &R[0][0]);
    float expected[] = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    TEST_ASSERT_TRUE(approx_equal(&R[0][0], expected, 16));
}

void test_FK_one_joint_equals_DH(void) {
    float params[4] = {(float)M_PI/2, 1.0f, 0.5f, 0};
    float DH[1][4] = {{(float)M_PI/2, 1.0f, 0.5f, 0}};
    float R_FK[4][4] = {};
    float R_DH[4][4] = {};
    robot.Forward_Kinematics(DH, 1, &R_FK[0][0]);
    robot.DH_to_HMatrix(params, &R_DH[0][0]);
    TEST_ASSERT_TRUE(approx_equal(&R_FK[0][0], &R_DH[0][0], 16));
}

void test_FK_two_translations_accumulate(void) {
    float DH[2][4] = {{0, 2, 0, 0}, {0, 3, 0, 0}};
    float R[4][4] = {};
    robot.Forward_Kinematics(DH, 2, &R[0][0]);
    float expected[] = {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 5,
        0, 0, 0, 1
    };
    TEST_ASSERT_TRUE(approx_equal(&R[0][0], expected, 16));
}

void test_FK_two_rotations_z_90_gives_180(void) {
    float DH[2][4] = {{(float)M_PI/2, 0, 0, 0}, {(float)M_PI/2, 0, 0, 0}};
    float R[4][4] = {};
    robot.Forward_Kinematics(DH, 2, &R[0][0]);
    float expected[] = {
        -1,  0, 0, 0,
         0, -1, 0, 0,
         0,  0, 1, 0,
         0,  0, 0, 1
    };
    TEST_ASSERT_TRUE(approx_equal(&R[0][0], expected, 16));
}

void test_FK_last_row_always_0001(void) {
    float DH[3][4] = {
        {1.1f, 0.5f, 0.3f, 0.2f},
        {0.8f, 1.0f, 0.1f, 0.5f},
        {0.3f, 0.2f, 0.4f, 1.1f}
    };
    float R[4][4] = {};
    robot.Forward_Kinematics(DH, 3, &R[0][0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, R[3][0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, R[3][1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0, R[3][2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1, R[3][3]);
}

void test_FK_six_dof_last_row_0001(void) {
    float DH[6][4] = {
        {0,        0.333f, 0,      (float)M_PI/2},
        {0,        0,      0.316f, 0},
        {0,        0.384f, 0,      (float)M_PI/2},
        {0,        0,      0,     -(float)M_PI/2},
        {0,        0,      0,      (float)M_PI/2},
        {0,        0.107f, 0,      0}
    };
    float R[4][4] = {};
    robot.Forward_Kinematics(DH, 6, &R[0][0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0, R[3][0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0, R[3][1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0, R[3][2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1, R[3][3]);
}

// ═════════════════════════════════════════════════════════════════════════
// INVERSE KINEMATICS TESTS
// ═════════════════════════════════════════════════════════════════════════

void test_IK_simple_forward_position(void) {
    // Target directly ahead: x=0.6, y=0 (both links extended)
    float x = 0.6f, y = 0.0f, z = 0.0f;
    float theta1, theta2, theta3;
    robot.Inverse_Kinematics(x, y, z, &theta1, &theta2, &theta3, true);
    
    // θ₁ should be 0 (pointing along X-axis)
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, theta1);
    // θ₂ should be valid (between -π and π)
    TEST_ASSERT_TRUE(theta2 >= -M_PI && theta2 <= M_PI);
    // All angles should be finite (no NaN/Inf)
    TEST_ASSERT_FALSE(isnan(theta1) || isnan(theta2) || isnan(theta3));
}

void test_IK_45_degree_position(void) {
    // 45° angle: x=y creates θ₁ = 45°
    float x = 0.5f, y = 0.5f, z = 0.0f;
    float theta1, theta2, theta3;
    robot.Inverse_Kinematics(x, y, z, &theta1, &theta2, &theta3, true);
    
    // θ₁ should be 45° (π/4 radians)
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, M_PI/4, theta1);
    // All angles should be finite
    TEST_ASSERT_FALSE(isnan(theta1) || isnan(theta2) || isnan(theta3));
}

void test_IK_elbow_up_vs_down(void) {
    // Same target, different elbow configurations
    float x = 0.4f, y = 0.3f, z = 0.0f;
    float theta1_up, theta2_up, theta3_up;
    float theta1_down, theta2_down, theta3_down;
    
    robot.Inverse_Kinematics(x, y, z, &theta1_up, &theta2_up, &theta3_up, true);
    robot.Inverse_Kinematics(x, y, z, &theta1_down, &theta2_down, &theta3_down, false);
    
    // θ₁ should be the same
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, theta1_up, theta1_down);
    // θ₂ should have opposite signs
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, theta2_up, -theta2_down);
}

void test_IK_unreachable_target(void) {
    // Target too far: r = √(0.8² + 0.6²) = 1.0, but L1+L2 = 0.6
    float x = 0.8f, y = 0.6f, z = 0.0f;
    float theta1, theta2, theta3;
    robot.Inverse_Kinematics(x, y, z, &theta1, &theta2, &theta3, true);
    
    // Should not produce NaN even though target is unreachable
    TEST_ASSERT_FALSE(isnan(theta1) || isnan(theta2) || isnan(theta3));
    // θ₂ will be clamped to acos(±1)
    TEST_ASSERT_TRUE(theta2 >= -M_PI/2 && theta2 <= M_PI/2);
}

void test_IK_origin(void) {
    // Target at origin - degenerate case
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float theta1, theta2, theta3;
    robot.Inverse_Kinematics(x, y, z, &theta1, &theta2, &theta3, true);
    
    // Should handle gracefully (no NaN)
    TEST_ASSERT_FALSE(isnan(theta1) || isnan(theta2) || isnan(theta3));
}

void test_IK_max_reach(void) {
    // Target at maximum reach: x = L1 + L2
    float x = robot.getLink1Length() + robot.getLink2Length(), y = 0.0f, z = 0.0f;
    float theta1, theta2, theta3;
    robot.Inverse_Kinematics(x, y, z, &theta1, &theta2, &theta3, true);
    
    // θ₂ should be close to 0 (links fully extended)
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, theta2);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, theta1);
}

// ═════════════════════════════════════════════════════════════════════════
// MAIN TEST RUNNER
// ═════════════════════════════════════════════════════════════════════════

int main(int argc, char **argv) {
    UNITY_BEGIN();

    // Matrix Multiplication Tests
    RUN_TEST(test_matmul_identity_times_identity);
    RUN_TEST(test_matmul_A_times_identity_equals_A);
    RUN_TEST(test_matmul_2x2_known_result);
    RUN_TEST(test_matmul_zero_matrix);
    RUN_TEST(test_matmul_homogeneous_translation);
    RUN_TEST(test_matmul_non_square_2x3_times_3x2);

    // DH to Homogeneous Matrix Tests
    RUN_TEST(test_DH_all_zeros_gives_identity);
    RUN_TEST(test_DH_rotation_z_90);
    RUN_TEST(test_DH_translation_z);
    RUN_TEST(test_DH_translation_x);
    RUN_TEST(test_DH_rotation_x_90);
    RUN_TEST(test_DH_last_row_always_0001);

    // Forward Kinematics Tests
    RUN_TEST(test_FK_zero_joints_gives_identity);
    RUN_TEST(test_FK_one_joint_equals_DH);
    RUN_TEST(test_FK_two_translations_accumulate);
    RUN_TEST(test_FK_two_rotations_z_90_gives_180);
    RUN_TEST(test_FK_last_row_always_0001);
    RUN_TEST(test_FK_six_dof_last_row_0001);

    // Inverse Kinematics Tests
    RUN_TEST(test_IK_simple_forward_position);
    RUN_TEST(test_IK_45_degree_position);
    RUN_TEST(test_IK_elbow_up_vs_down);
    RUN_TEST(test_IK_unreachable_target);
    RUN_TEST(test_IK_origin);
    RUN_TEST(test_IK_max_reach);

    return UNITY_END();
}
