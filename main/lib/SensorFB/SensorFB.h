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
    void SensorFB();

    // initialization
    void init();
    void set_initial_rot();
    void update();

    array<float, 2> com_pos_fb();
    array<float, 2> foot_pos_fb();
    
    // body inclination feedback
    array<float, 2> angle_com_pos_fb();
    void angle_foot_pos_fb();
    array<float, 3> vd_fb(array<float, 3> vd);
    
    // acceleration feedback
    float acc_delay_duration_fb(int step);
    array<float, 2> acc_foot_pos_fb();
    array<float, 2> acc_com_pos_fb();

private:
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

    // initial rotation
    imu::Quaternion q_ref;
    // data
    imu::Vector<3> euler;
    imu::Vector<3> gyro;
    imu::Vector<3> acc;

    // initial rotation
    array<float, 3> euler_initial;


};