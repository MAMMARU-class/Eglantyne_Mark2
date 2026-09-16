#include "velocity_control.h"

#include <cmath>

void VelocityControl::set_x_velocity_at_reference(
    float velocity_at_reference)
{
    this->x_velocity_at_reference = velocity_at_reference;
}

void VelocityControl::set_y_velocity_at_reference(
    float velocity_at_reference)
{
    this->y_velocity_at_reference = velocity_at_reference;
}

void VelocityControl::set_yaw_target_deg(float yaw_target_deg){
    this->yaw_target_deg = normalize_angle_deg(yaw_target_deg);
}

void VelocityControl::set_yaw_pid_gains(float kp, float ki, float kd){
    this->yaw_kp = kp;
    this->yaw_ki = ki;
    this->yaw_kd = kd;
}

void VelocityControl::set_yaw_pd_gains(float kp, float kd){
    set_yaw_pid_gains(kp, 0.0f, kd);
}

float VelocityControl::calculate_x_velocity(float t_sup_s) const{
    if (t_sup_s <= 0.0f){
        return 0.0f;
    }

    return this->x_velocity_at_reference * REFERENCE_T_SUP_S / t_sup_s;
}

float VelocityControl::calculate_y_velocity(float t_sup_s) const{
    if (t_sup_s <= 0.0f){
        return 0.0f;
    }

    return this->y_velocity_at_reference * REFERENCE_T_SUP_S / t_sup_s;
}

float VelocityControl::calculate_yaw_velocity(float yaw_deg, float dt_s){
    const float error_deg = normalize_angle_deg(
        this->yaw_target_deg - yaw_deg);

    float error_rate_deg_s = 0.0f;
    if (this->yaw_feedback_initialized && dt_s > 0.0f){
        error_rate_deg_s = normalize_angle_deg(
            error_deg - this->yaw_error_last_deg) / dt_s;
    }

    if (dt_s > 0.0f){
        const float integral_candidate =
            this->yaw_error_integral_deg_s + error_deg * dt_s;
        const float correction_candidate =
            this->yaw_kp * error_deg +
            this->yaw_ki * integral_candidate +
            this->yaw_kd * error_rate_deg_s;

        // Do not accumulate further when the normalized yaw command is
        // saturated in the direction in which the current error would push it.
        const bool winding_up_positive =
            correction_candidate > 1.0f && error_deg > 0.0f;
        const bool winding_up_negative =
            correction_candidate < -1.0f && error_deg < 0.0f;
        if (!winding_up_positive && !winding_up_negative){
            this->yaw_error_integral_deg_s = integral_candidate;
        }
    }

    this->yaw_error_last_deg = error_deg;
    this->yaw_feedback_initialized = true;

    const float output =
        -1 * (
            this->yaw_kp * error_deg +
            this->yaw_ki * this->yaw_error_integral_deg_s +
            this->yaw_kd * error_rate_deg_s);

    // GaitController interprets vd[2] as a normalized rotation command.
    return clamp(output, -1.0f, 1.0f);
}

void VelocityControl::reset_yaw_feedback(){
    this->yaw_error_last_deg = 0.0f;
    this->yaw_error_integral_deg_s = 0.0f;
    this->yaw_feedback_initialized = false;
}

float VelocityControl::normalize_angle_deg(float angle_deg){
    float normalized = std::fmod(angle_deg + 180.0f, 360.0f);
    if (normalized < 0.0f){
        normalized += 360.0f;
    }
    return normalized - 180.0f;
}

float VelocityControl::clamp(
    float value, float minimum, float maximum)
{
    if (value < minimum){
        return minimum;
    }
    if (value > maximum){
        return maximum;
    }
    return value;
}
