#include "SensorFB.h"

SensorFB::SensorFB(){}

void SensorFB::init(){
    Wire.begin(SDA, SCL);

    if (!bno.begin()){
        Serial.print("No BNO055 detected");
        while(1);
    }
    delay(100);
    bno.setExtCrystalUse(true);

    Serial.println("BNO055 initialized");    
}

void SensorFB::set_initial_rot(){
    this->q_ref = bno.getQuat();
    this->q_ref.normalize();

    Serial.println("set initial rotation");
}

void SensorFB::update(){
    // current pose
    imu::Quaternion q_now = bno.getQuat();
    q_now.normalize();
    // relative pose
    imu::Quaternion q_rel = this->q_ref.conjugate() * q_now;
    q_rel.normalize();
    // Euler
    this->euler = q_rel.toEuler();

    // gyro
    imu::Quaternion<3> gyro_now = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    this->gyro = q_rel.rotate_vec(gyro_now);

    // acceleration
    imu::Vector<3> accel_body = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    // convert to world frame
    this->acc = q_rel.rotateVector(accel_body);
}


