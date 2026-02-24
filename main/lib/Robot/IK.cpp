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
    angles[3] = 2.0f * gamma;   // leg_under
    angles[4] = gamma - beta;          // foot_pitch

    return angles;
}

array<float, 3> Robot::arm_k_solver(array<float, 3> arm_angles)
{
    float c, s;

    c = cos(arm_angles[0]);
    s = sin(arm_angles[0]);
    array<array<float, 3>, 3> R1 = {{
        {c, 0, -s},
        {0, 1, 0},
        {s, 0, c}
    }};

    c = cos(arm_angles[1]);
    s = sin(arm_angles[1]);
    array<array<float, 3>, 3> R2 = {{
        {1, c, -s},
        {0, s, c},
        {0, 0, 1}
    }};

    c = cos(PI / 4.0f);
    s = sin(PI / 4.0f);
    array<array<float, 3>, 3> R3 = {{
        {c, -s, 0},
        {s, c, 0},
        {0, 0, 1}
    }};

    c = cos(arm_angles[2]);
    s = sin(arm_angles[2]);
    array<array<float, 3>, 3> R4 = {{
        {c, 0, -s},
        {0, 1, 0},
        {s, 0, c}
    }};

    float l1 = this->l_arm_upper;
    float l2 = this->l_arm_lower;

    array<array<float, 3>, 3> R12 = mul_rot_matrices(R1, R2);
    array<array<float, 3>, 3> R123 = mul_rot_matrices(R12, R3);
    array<array<float, 3>, 3> R1234 = mul_rot_matrices(R123, R4);

    array<float, 3> arm_pos = {
        -l1 * R12[0][2] - l2 * R1234[0][0],
        -l1 * R12[1][2] - l2 * R1234[1][0],
        -l1 * R12[2][2] - l2 * R1234[2][0]
    };

    return {arm_pos[0] / 1000.0f, arm_pos[1] / 1000.0f, arm_pos[2] / 1000.0f};
}

array<array<float, 3>, 3> Robot::mul_rot_matrices(array<array<float, 3>, 3> mat1, array<array<float, 3>, 3> mat2){
    array<array<float, 3>, 3> result = {0};
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            for (int k=0; k<3; k++){
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
    return result;
}

array<float, 3> Robot::rot_mat_vector(array<array<float, 3>, 3> T, array<float, 3> vec){
    array<float, 3> result = {0};
    for (int i=0; i<3; i++){
        for (int j=0; j<3; j++){
            result[i] += T[i][j] * vec[j];
        }
    }
    return {result[0], result[1], result[2]};
};
