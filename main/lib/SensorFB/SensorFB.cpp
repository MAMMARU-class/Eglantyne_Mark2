#include "SensorFB.h"

SensorFB::SensorFB(){}

void SensorFB::init(){
    Serial.println("Initializing BNO055...");

    if (!Wire.begin(SDA, SCL)) {
        Serial.println("Failed to initialize I2C");
        while(1);
    }
    Serial.println("Wire initialized");

    if (!bno.begin()){
        Serial.print("No BNO055 detected");
        while(1);
    }
    delay(100);
    bno.setExtCrystalUse(true);

    delay(500);
    this->euler_last = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    this->acc_last   = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    update();
    Serial.println("BNO055 initialized");
}

void SensorFB::update(){
    // current pose
    this->euler_last = this->euler;
    this->euler      = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    // acceleration
    this->acc_last   = this->acc;
    this-> acc       = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}

// state check
bool SensorFB::fall(){
    float fall_angle = 30.0f;
    if(this->euler.y() > -fall_angle && this->euler.y() < fall_angle && 
       this->euler.z() > -fall_angle && this->euler.z() < fall_angle){
        return false;
    }else{
        return true;
    }
}
bool SensorFB::face_up(){
    if(-this->euler.y()<0){
        return true;
    }else{
        return false;
    }
}

// feedback functions
// body inclination feedback
array<float, 2> SensorFB::angle_com_pos_fb(){
    // calculate angle error
    float err = -this->euler.y();
    float err_last = -this->euler_last.y();
    float derr = err - err_last;

    // ocnvert angle to com position
    array<float, 2> angle_com_err = {
        sinf(err * PI / 180.0f) * this->l_pivot2com,
        // sinf( this->euler.z() * PI / 180.0f) * this->l_pivot2com
        0
    };
    array<float, 2> angle_com_derr = {
        sinf(derr * PI / 180.0f) * this->l_pivot2com,
        // sinf( this->gyro.z() * PI / 180.0f) * this->l_pivot2com
        0
    };

    // feedback output
    array<float, 2> angle_com_fb = {
        this->kp_angle_com * angle_com_err[0] + this->kd_angle_com * angle_com_derr[0],
        this->kp_angle_com * angle_com_err[1] + this->kd_angle_com * angle_com_derr[1]
    };

    return angle_com_fb;
    // return {0,0};
}

float SensorFB::angle_phi_fb(){
    // rotate body base roll angle accordance with body angle.
    // calculate angle error
    float err = -this->euler.y();
    float err_last = -this->euler_last.y();
    float derr = err - err_last;

    err = err * PI / 180.0f;
    derr = derr * PI / 180.0f;

    float angle_phi_fb = this->kp_phi_body * err + this->kd_phi_body * derr;
    return angle_phi_fb;
}

array<float, 3> SensorFB::vd_fb(array<float, 3> vd){
    array<float, 3> vd_fb = {
        kp_angle_vd * float(-this->euler.y()) + kd_angle_vd * float(-this->gyro.y()),
        kp_angle_vd * float( this->euler.z()) + kd_angle_vd * float( this->gyro.z()),
        0
    };
    return vd_fb;
}

// acceleration feedback
int SensorFB::update_rate_fb(array<float, 3> approx_coeff, array<float, 2> ideal_acc, float Tc, float t_ideal, int update_rate, float com_pos){
    // update last ideal_acc
    float acc = this->acc.y();
    float acc_last = this->acc_last.y();
    this->ideal_acc_last = ideal_acc[1];
    
    float jerk_abs = abs(acc) - abs(acc_last);

    // approximated trajectory: y = a*t^2 + b*t + c
    float a = approx_coeff[0];
    float b = approx_coeff[1];
    float c = approx_coeff[2];
    // move trajectory to reduce single term: y = a*(t-t_mid)^2 + c_dash
    float c_dash = a * (b*b)/(4*a*a) - b * b/(2*a) + c;
    float t_mid = -b/(2*a);
    t_ideal = t_ideal - t_mid;

    // time signiture
    int sig;
    if (jerk_abs > 0){
        sig = -1;
    }else{
        sig = 1;
    }

    // estimate current pos and phase(time) based on current acceleration.
    float pos_y = acc * (Tc*Tc);
    if(c_dash > 0 && pos_y < c_dash){
        pos_y = c_dash;
    }else if (c_dash < 0 && pos_y > c_dash){
        pos_y = c_dash;
    }
    float t_now;
    t_now = sqrt((pos_y - c_dash)/a) * sig;

    float t_err = t_now - t_ideal;
    float t_derr = t_err - this->t_err_last;
    this->t_err_last = t_err;

    float acc_fb = abs(this->kp_update_rate * t_err + this->kd_update_rate * t_derr) + 1.0f;
    // Serial.print("acc_fb: "); Serial.println(acc_fb, 4);

    // return update rate
    float update_rate_fb;
    if (t_err > 0){
        // delay. fastern phase velocity.
        update_rate_fb = update_rate * acc_fb;
    }else{
        // advance. slower phase velocity.
        update_rate_fb = update_rate / acc_fb;
    }

    // cast to int, and handle 0
    int update_rate_fb_int = (int)update_rate_fb;
    if (update_rate_fb_int == 0){
        update_rate_fb_int = 1;
    }
    Serial.println();
    // Serial.print("a: "); Serial.print(a, 4); Serial.print(", b: "); Serial.print(b, 4); Serial.print(", c: "); Serial.println(c, 4);
    // Serial.print("acc_ideal: "); Serial.print(ideal_acc[1], 4); Serial.print(", acc: "); Serial.println(acc, 4);
    // Serial.print("ideal y: "); Serial.print(com_pos, 4); Serial.print(", calculated y: "); Serial.println(a * t_ideal * t_ideal + c_dash, 4);
    // Serial.print(", pos_y: "); Serial.println(pos_y, 4);
    // Serial.print("t_ideal: "); Serial.print(t_ideal, 4); Serial.print(", calculated t: "); Serial.println(sqrt((com_pos - c_dash)/a) * sig, 4);
    // Serial.print("t_now: "); Serial.println(sqrt((pos_y - c_dash)/a) * sig, 4);
    // Serial.print("t_err: "); Serial.println(t_err, 4);
    // Serial.print("update_rate_fb: "); Serial.println(update_rate_fb, 4);
    return update_rate_fb_int;
}

array<float, 2> SensorFB::pn_dot_pn_fb(){
}
