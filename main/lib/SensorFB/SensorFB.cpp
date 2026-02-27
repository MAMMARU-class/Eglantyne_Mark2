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
    delay(1000);
    update();
    init_norm();
}

void SensorFB::update(){
    // current pose
    this->euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    // Serial.print("euler: "); Serial.print(this->euler.x(), 4); Serial.print(", "); Serial.print(this->euler.y(), 4); Serial.print(", "); Serial.println(this->euler.z(), 4);

    // gyro
    this->gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    // acceleration
    this-> acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    this->euler_norm.x() = this->euler_norm.x() * 0.3 + this->euler.x() * 0.7;
    this->euler_norm.y() = this->euler_norm.y() * 0.3 + this->euler.y() * 0.7;
    this->euler_norm.z() = this->euler_norm.z() * 0.3 + this->euler.z() * 0.7;
    this->gyro_norm.x() = this->gyro_norm.x() * 0.3 + this->gyro.x() * 0.7;
    this->gyro_norm.y() = this->gyro_norm.y() * 0.3 + this->gyro.y() * 0.7;
    this->gyro_norm.z() = this->gyro_norm.z() * 0.3 + this->gyro.z() * 0.7;
}

void SensorFB::init_norm(){
    this->euler_norm = this->euler;
    this->gyro_norm = this->gyro;
}

// state check
bool SensorFB::fall(){
    if(this->euler.y() > -25 && this->euler.y() < 25 && this->euler.z() > -25 && this->euler.z() < 25){
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
    float err = -this->euler.y();
    float derr = err - this->angle_com_err_last;
    this->angle_com_err_last = err;
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

    // Serial.print("angle_com_err: "); Serial.print(angle_com_err[0], 4); Serial.print(", "); Serial.println(angle_com_err[1], 4);

    // feed back output
    array<float, 2> angle_com_fb = {
        this->kp_angle_com * angle_com_err[0] + this->kd_angle_com * angle_com_derr[0],
        this->kp_angle_com * angle_com_err[1] + this->kd_angle_com * angle_com_derr[1]
    };

    return angle_com_fb;
    // return {0,0};
}

array<float, 2> SensorFB::angle_foot_pos_fb(){
    array<float, 2> foot_pos_fb = {
        kp_angle_foot * float(-this->euler_norm.y()) + kd_angle_foot * float(this->gyro_norm.x()),
        kp_angle_foot * float(this->euler_norm.z()) + kd_angle_foot * float(this->gyro_norm.y())
    };
    return foot_pos_fb;
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
array<float, 2> SensorFB::acc_com_pos_fb(){
    return {0,0};
}

array<float, 2> SensorFB::acc_foot_pos_fb(){
    float d_delay_duration = this->delay_duration - this->delay_duration_last;
    this->delay_duration_last = this->delay_duration;
    array<float, 2> foot_pos_fb = {
        kp_acc_foot * this->delay_duration + kd_acc_foot * d_delay_duration,
        0
    };
    return foot_pos_fb;
}

float SensorFB::delay_duration_fb(array<float, 3> approx_coeff, array<float, 2> ideal_acc, float Tc, float t_ideal, int step){
    // check if com already pass the top
    float ideal_jerk_abs = abs(ideal_acc[1]) - abs(this->ideal_acc_last);
    this->ideal_acc_last = ideal_acc[1];
    
    float jerk_abs = abs(this->acc.z()) - abs(this->acc_last);
    this->acc_last = this->acc.z();

    // float a = approx_coeff[0];
    // float b = approx_coeff[1];
    // float c = approx_coeff[2];
    // c = a * (b*b)/(4*a*a) - b * b/(2*a) + c;
    // float t_mid = -b/(2*a);
    // t_ideal = t_ideal - t_mid;

    // int sig;
    // if (jerk_abs > 0){
    //     sig = 1;
    // }else{
    //     sig = -1;
    // }

    // float pos_y = acc.z() / (Tc*Tc);
    // float t_now;
    // if ((pos_y - c)/a > 0){
    //     t_now = sqrt((pos_y - c)/a) * sig;
    // }else{
    //     t_now = t_ideal;
    // }

    // float t_err = t_now - t_ideal;
    // float t_derr = t_err - this->t_err_last;
    // this->t_err_last = t_err;
    // // Serial.print("t_err: "); Serial.print(t_err, 4); Serial.print(", t_derr: "); Serial.println(t_derr, 4);

    // float acc_fb;
    // // Serial.print("acc_fb: "); Serial.println(acc_fb, 4);

    // // return delay duration in ms
    // if (t_err > 0){
    //     acc_fb = abs(kp_acc_delay * t_err + kd_acc_delay * t_derr) + 1.0f;
    //     this->delay_duration = 1000.0f / step / acc_fb;
    // }else{
    //     acc_fb = abs(kp_acc_delay * t_err + kd_acc_delay * t_derr) + 1.0f;
    //     this->delay_duration = 1000.0f / step * acc_fb;
    // }
    // Serial.print("delay_duration: "); Serial.println(this->delay_duration, 4);
    // return this->delay_duration;

    float fb_mag;
    if (abs(ideal_jerk_abs) < 0.01){
        fb_mag = 1;
    }else{
        fb_mag = jerk_abs / ideal_jerk_abs;
    }
    if (fb_mag == 0){fb_mag = 1;}
    float fb_dir = fb_mag / abs(fb_mag);

    // calculate error
    float err = abs(ideal_acc[1]) - abs(this->acc.z());
    err *= fb_dir;
    float derr = err - this->acc_err_last;
    this->acc_err_last = err;

    // feed back coefficient
    float acc_fb_coeff = (this->kp_acc_delay * err + this->kd_acc_delay * derr);
    // Serial.print("acc_fb_coeff: "); Serial.println(acc_fb_coeff, 4);

    // normalize into 0-2
    float acc_fb = 0.95 * (2.0f / PI * atan(acc_fb_coeff) + 1.0f);
    // Serial.println("acc_fb_coeff: " + String(acc_fb_coeff, 4));
    // Serial.println("acc_fb: " + String(acc_fb, 4));

    // show error as color
    show_acc_error(acc_fb);

    this->delay_duration = 1000.0f / step * acc_fb;
    return this->delay_duration;
}

void SensorFB::show_acc_error(float err){
    array<float, 3> color;

    float color_val = err - 1.0f;
    // Serial.print("acc error: "); Serial.println(color_val, 4);
    if (color_val >= 0) {
        uint32_t r = (uint32_t)(color_val * 255);
        color[0] = r;
        color[1] = 0;
        color[2] = 255 - r;
    } else {
        uint32_t g = (uint32_t)(-color_val * 255);
        color[0] = 0;
        color[1] = g;
        color[2] = 255 - g;
    }
    // neopixelWrite(RGB_BUILTIN, color[0], color[1], color[2]);
}

// integrate feedback
array<float, 2> SensorFB::foot_pos_fb(){
    array<float, 2> angle_fb = angle_foot_pos_fb();
    array<float, 2> acc_fb = acc_foot_pos_fb();

    array<float, 2> foot_pos_fb = {
        angle_fb[0] + acc_fb[0],
        angle_fb[1] + acc_fb[1]
    };
    // return foot_pos_fb;
    return {0,0};
}
