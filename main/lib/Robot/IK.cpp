#include "Robot.h"
#include <cmath>

array<float, 6> Robot::leg_ik_solver_phi_zero(array<float, 3> foot2com, float theta, bool is_right)
{
    float dir = 1.0;
    if(!is_right){dir = -1.0;}

    array<float, 6> angles = {0};

    // foot2com = [x, y, z]
    float com_x = foot2com[0] * 1000; // mm
    float com_y = foot2com[1] * 1000; // mm
    float com_z = foot2com[2] * 1000; // mm

    /* -------------------------
     * rotate (yaw compensation)
     * ------------------------- */
    float foot2leg_yaw_x = com_x + dir*l_com_y * sin(theta);
    float foot2leg_yaw_y = com_y - dir*l_com_y * cos(theta);

    // leg yaw
    angles[0] = theta;

    /* -------------------------
     * back (roll)
     * ------------------------- */
    float alpha = atan2(
        foot2leg_yaw_y,
        com_z - (l_roll_com + l_foot_z)
    );

    // leg roll & foot roll
    angles[1] = alpha;
    angles[5] = alpha;

    /* -------------------------
     * side (pitch direction)
     * ------------------------- */
    float arctan_x =
        com_z - (l_roll_com + l_foot_z + l_roll2pitch * cos(alpha));
    float arctan_y =
        -foot2leg_yaw_x - l_foot_x;

    float beta = atan2(arctan_y, arctan_x);

    /* -------------------------
     * knee (2-link geometry)
     * ------------------------- */
    float l_leg_all = sqrt( arctan_y*arctan_y + arctan_x*arctan_x ) / cos(alpha) / cos(beta);

    // safety clamp for acos
    float cos_gamma = (l_leg_all / 2.0f) / l_leg;
    cos_gamma = constrain(cos_gamma, -1.0f, 1.0f);

    float gamma = acos(cos_gamma);

    /* -------------------------
     * joint assignment
     * ------------------------- */
    angles[2] = gamma + beta;          // leg_upper
    angles[3] = 2.0f * gamma;          // leg_under
    angles[4] = gamma - beta;          // foot_pitch

    return angles;
}

array<float, 3> Robot::arm_k_solver(array<float, 3> arm_angles)
{
    float c, s;
    float l1 = this->l_arm_upper;
    float l2 = this->l_arm_lower;

    c = cos(arm_angles[0]);
    s = sin(arm_angles[0]);
    array<array<float, 4>, 4> T1 = {{
        {c, 0, -s, 0},
        {0, 1, 0, 0},
        {s, 0, c, 0},
        {0, 0, 0, 1}
    }};

    c = cos(arm_angles[1]);
    s = sin(arm_angles[1]);
    array<array<float, 4>, 4> T2 = {{
        {1, 0, 0, 0},
        {0, c, -s, 0},
        {0, s, c, 0},
        {0, 0, 0, 1}
    }};

    c = cos(PI / 4.0f);
    s = sin(PI / 4.0f);
    array<array<float, 4>, 4> T3 = {{
        {c, -s, 0, 0},
        {s, c, 0, 0},
        {0, 0, 1, -l1},
        {0, 0, 0, 1}
    }};

    c = cos(arm_angles[2]);
    s = sin(arm_angles[2]);
    array<array<float, 4>, 4> T4 = {{
        {c, 0, -s, 0},
        {0, 1, 0, 0},
        {s, 0, c, 0},
        {0, 0, 0, 1}
    }};

    array<array<float, 4>, 4> T5 = {{
        {1,0,0,0},
        {0,1,0,0},
        {0,0,1,-l2},
        {0,0,0,1}
    }};

    array<array<float, 4>, 4> R12 = mul_T_matrices(T1, T2);
    array<array<float, 4>, 4> R12345 = mul_T_matrices(mul_T_matrices(R12, T3), mul_T_matrices(T4, T5));

    array<float, 3> arm_pos = {
        R12345[0][3],
        R12345[1][3],
        R12345[2][3]
    };

    return {arm_pos[0] / 1000.0f, arm_pos[1] / 1000.0f, arm_pos[2] / 1000.0f};
}

array<array<float, 4>, 4> Robot::mul_T_matrices(array<array<float, 4>, 4> mat1, array<array<float, 4>, 4> mat2){
    array<array<float, 4>, 4> result = {0};
    for (int i=0; i<4; i++){
        for (int j=0; j<4; j++){
            for (int k=0; k<4; k++){
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
    return result;
}
