#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define SDA 5
#define SCL 4

using std::array;

struct BNO055Data {
    array<float, 3> acceleration;
    array<float, 3> angle;
    array<float, 3> angular_velocity;
};

class SensorFB{
public:
    SensorFB();

    // initialization
    void init();
    void update();

    BNO055Data get_bno055_data() const;
    float get_last_pos_y() const { return this->last_pos_y; }
    float get_last_update_rate_fb() const { return this->last_update_rate_fb; }

    // state check
    bool fall();
    bool face_up();

    // feedback
    float angle_phi_fb();

    // acceleration feedback
    int update_rate_fb(
        float t_ideal, array<float, 2> acc_ideal,
        array<float, 3> approx_coeff, float Tc, int update_rate,
        float com_pos);
    array<float, 2> x0_vx0_fb(float tx, float x0, float vx0, float Tc, int control_step, float com_x_pos);

    // setters
    void set_update_rate_fb_gains(float kp, float kd){ this->kp_update_rate = kp; this->kd_update_rate = kd; }
    void set_x0_vx0_fb_gains(float kp, float kd){ this->kp_x0_vx0 = kp; this->kd_x0_vx0 = kd; }
private:
    static constexpr float ACCEL_LPF_CUTOFF_HZ = 2.0f;
    static constexpr float GYRO_LPF_CUTOFF_HZ = 2.0f;
    static constexpr float ANGLE_LPF_CUTOFF_HZ = 1.0f;

    static float low_pass_filter(
        float input, float previous, float cutoff_hz, float dt_s);
    static float normalize_angle_deg(float angle_deg);
    static float low_pass_angle_deg(
        float input, float previous, float cutoff_hz, float dt_s);

    // bno
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
    imu::Vector<3> euler_last;
    imu::Vector<3> euler;
    imu::Vector<3> acc_last;
    imu::Vector<3> acc;
    imu::Vector<3> gyro;
    unsigned long last_bno_update_us = 0;
    bool bno_filter_initialized = false;

    // gains
    float kp_phi_body    = 0.45f;
    float kd_phi_body    = 0.015f;

    // successive gains
    // float kp_update_rate = 7.0f;
    // float kd_update_rate = 0.5f;
    // testing
    float kp_update_rate = 2.5f;
    float kd_update_rate = 1.8f;
    // float kp_update_rate = 120.0f;
    // float kd_update_rate = 50.0f;
    // float kp_update_rate = 1.0f;
    // float kd_update_rate = 0.1f;

    float kp_x0_vx0 = 0.0027f;
    float kd_x0_vx0 = 0.000005f;
    // float kp_x0_vx0 = 0.0f;
    // float kd_x0_vx0 = 0.0f;

    float a_pos = 1.0f;
    float a_vel = 0.007f;

    // feedback state variables
    // update rate feedback
    float last_update_rate_fb = NAN;
    float acc_ideal_last = 0.0f;
    float t_err_last     = 0.0f;
    float last_pos_y = NAN;
    float fb = NAN;

    // x0 and vx0 feedback
    float x0_fb_last  = 0.0f;
    float vx0_fb_last = 0.0f;

};
