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
    float l1 = this->l_arm_upper;
    float l2 = this->l_arm_lower;
    float theta1 = arm_angles[0];
    float theta2 = arm_angles[1];
    float theta4 = arm_angles[2];
    float theta3 = M_PI / 4.0f; // 45 degrees

    // T1: rotation around Z
    float c1 = cos(theta1), s1 = sin(theta1);
    // T2: rotation around Y
    float c2 = cos(-theta2), s2 = sin(-theta2);
    // T3: rotation around Z + translation
    float c3 = cos(theta3), s3 = sin(theta3);
    // T4: rotation around Z
    float c4 = cos(theta4), s4 = sin(theta4);

    // Combined transformation matrix multiplication result
    // Extract final position from T = T1 @ T2 @ T3 @ T4 @ T5
    float x = -l1*s3*c1 - l2*(s3*c1*c4 - s1*s4);
    float y = l1*c3*s2 + l2*(c3*s2*c4 + c2*s4);
    float z = -l1*c3 - l2*c3*c4;

    return array<float, 3>{x*0.001f, y*0.001f, z*0.001f}; // convert to meters
}
