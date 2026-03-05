#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define SDA 5
#define SCL 4

using std::array;

class SensorFB{
public:
    SensorFB();

    // initialization
    void init();
    void update();
    void set_phi(float phi){ this->phi = phi; };

    // state check
    bool fall();
    bool face_up();
    bool fly();
    bool hit_ground();

    // feedback
    // body inclination feedback
    array<float, 2> angle_com_pos_fb();
    float angle_phi_fb();
    array<float, 3> vd_fb(array<float, 3> vd);

    // acceleration feedback
    int update_rate_fb(
        float t_ideal, array<float, 2> acc_ideal,
        array<float, 3> approx_coeff, float Tc, int update_rate,
        float com_pos);
    array<float, 2> x0_vx0_fb(float tx, float x0, float vx0, float Tc, int control_step, float com_x_pos);

    // getters
    float get_l_pivot2com(){ return l_pivot2com; }

private:
    // bno
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
    imu::Vector<3> euler_last;
    imu::Vector<3> euler;
    imu::Vector<3> acc_last;
    imu::Vector<3> acc;

    float phi = 0.0f;

    // body param
    float l_pivot2com = 0.07;

    // gains
    float kp_angle_com   = 0.3f;
    float kd_angle_com   = 0.001f;

    float kp_phi_body    = 0.65f;
    float kd_phi_body    = 0.01f;

    float kp_angle_vd    = 0.001f;
    float kd_angle_vd    = 0.0f;

    // float kp_update_rate = 5.5f;
    float kp_update_rate = 7.0f;
    float kd_update_rate = 0.5f;

    // float kp_x0_vx0 = 0.0015f;
    // float kd_x0_vx0 = 0.00005f;
    float kp_x0_vx0 = 0.0008f;
    float kd_x0_vx0 = 0.00002f;

    float a_pos = 1.0f;
    float a_vel = 0.007f;

    // feedback state variables
    // update rate feedback
    float acc_ideal_last = 0.0f;
    float t_err_last     = 0.0f;

    // x0 and vx0 feedback
    float x0_fb_last  = 0.0f;
    float vx0_fb_last = 0.0f;
};
