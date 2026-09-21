#ifndef VELOCITY_CONTROL_H
#define VELOCITY_CONTROL_H

class VelocityControl {
public:
    static constexpr float REFERENCE_T_SUP_S = 0.14f;

    // velocity_at_reference is the desired velocity at T_sup = 0.14 s.
    void set_x_velocity_at_reference(float velocity_at_reference);
    void set_y_velocity_at_reference(float velocity_at_reference);

    // The yaw target can be changed by an external command later.
    void set_yaw_target_deg(float yaw_target_deg);
    void set_yaw_pid_gains(float kp, float ki, float kd);
    // Compatibility helper for callers that intentionally disable I control.
    void set_yaw_pd_gains(float kp, float kd);

    float calculate_x_velocity(float t_sup_s) const;
    float calculate_y_velocity(float t_sup_s) const;
    float calculate_yaw_velocity(float yaw_deg, float dt_s);

    void reset_yaw_feedback();

private:
    static float normalize_angle_deg(float angle_deg);
    static float clamp(float value, float minimum, float maximum);

    float x_velocity_at_reference = 0.0f;
    float y_velocity_at_reference = 0.0f;
    float yaw_target_deg = 0.0f;

    // vd[2] is a normalized rotation command, so these gains produce a
    // dimensionless output from degree, degree-second, and degree/second
    // errors.
    float yaw_kp = 0.002f;
    float yaw_ki = 0.00005f;
    float yaw_kd = 0.000001f;

    float yaw_error_last_deg = 0.0f;
    float yaw_error_integral_deg_s = 0.0f;
    bool yaw_feedback_initialized = false;
};

#endif
