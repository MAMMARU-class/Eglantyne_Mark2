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

    // state check
    bool fall();
    bool face_up();

    // feedback
    // body inclination feedback
    array<float, 2> angle_com_pos_fb();
    float angle_phi_fb();
    array<float, 3> vd_fb(array<float, 3> vd);

    // acceleration feedback
    int update_rate_fb(array<float, 3> approx_coeff, array<float, 2> ideal_acc, float Tc, float t_ideal, int update_rate, float com_pos);
    array<float, 2> pn_dot_pn_fb();

private:
    // bno
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
    imu::Vector<3> euler;
    imu::Vector<3> gyro;
    imu::Vector<3> acc;

    // body param
    float l_pivot2com = 0.07;

    // gains
    float kp_angle_com = 0.1f;
    float kd_angle_com = 0.001f;

    float kp_phi_body = 0.55f;
    float kd_phi_body = 0.01f;

    float kp_angle_vd = 0.001f;
    float kd_angle_vd = 0.0f;

    float kp_update_rate_SINGLE = 5.0f;
    float kd_update_rate_SINGLE = 0.5f;

    // feedback state variables
    float angle_err_last = 0.0f;

    float ideal_acc_last = 0.0f;
    float acc_last = 0.0f;
    float t_err_last = 0.0f;
    float acc_err_last = 0.0f;

    float delay_duration = 0.0f;
    float delay_duration_last = 0.0f;
};
