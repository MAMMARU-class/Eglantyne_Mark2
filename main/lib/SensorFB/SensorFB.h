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
    void set_initial_rot();
    void update();
    void init_norm();

    // state check
    bool fall();
    bool face_up();

    // feedback
    // body inclination feedback
    array<float, 2> angle_com_pos_fb();
    array<float, 2> angle_foot_pos_fb();
    array<float, 3> vd_fb(array<float, 3> vd);

    // acceleration feedback
    array<float, 2> acc_com_pos_fb();
    array<float, 2> acc_foot_pos_fb();
    float delay_duration_fb(array<float, 2> ideal_acc, int step);
    void show_acc_error(float err);
    // integrate feedback
    array<float, 2> foot_pos_fb();

private:
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

    // initial rotation
    imu::Quaternion q_ref;
    bool initialized = false;
    // data
    imu::Vector<3> euler;
    imu::Vector<3> gyro;
    imu::Vector<3> acc;

    imu::Vector<3> euler_norm;
    imu::Vector<3> gyro_norm;

    // initial rotation
    array<float, 3> euler_initial;

    // body param
    float l_pivot2com = 0.07;

    // gains
    float kp_angle_com = 0.4f;
    float kd_angle_com = 0.01f;
    // float kp_angle_com = 0.0f;
    // float kd_angle_com = 0.0f;

    float kp_angle_foot = 0.01f;
    float kd_angle_foot = 0.001f;

    float kp_angle_vd = 0.001f;
    float kd_angle_vd = 0.0f;

    float kp_acc_delay = 3.0f;
    float kd_acc_delay = 0.0f;

    float kp_acc_com = 0.01f;
    float kd_acc_com = 0.001f;
    
    float kp_acc_foot = 0.01f;
    float kd_acc_foot = 0.001f;

    // feedback state variables
    float ideal_acc_last = 0.0f;
    float acc_last = 0.0f;
    float acc_err_last = 0.0f;

    float delay_duration = 0.0f;
    float delay_duration_last = 0.0f;
};
