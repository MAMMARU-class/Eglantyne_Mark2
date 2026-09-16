#include "SensorFB.h"

SensorFB::SensorFB(){}

// data filtering functions
float SensorFB::low_pass_filter(
    float input, float previous, float cutoff_hz, float dt_s)
{
    const float alpha = 1.0f - expf(-2.0f * PI * cutoff_hz * dt_s);
    return previous + alpha * (input - previous);
}

float SensorFB::normalize_angle_deg(float angle_deg){
    float normalized = fmodf(angle_deg + 180.0f, 360.0f);
    if (normalized < 0.0f){
        normalized += 360.0f;
    }
    return normalized - 180.0f;
}

float SensorFB::low_pass_angle_deg(
    float input, float previous, float cutoff_hz, float dt_s)
{
    const float angle_difference = normalize_angle_deg(input - previous);
    return normalize_angle_deg(
        low_pass_filter(angle_difference, 0.0f, cutoff_hz, dt_s) + previous);
}

// initialization
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
    this->last_bno_update_us = micros();
    this->bno_filter_initialized = false;
    update();
    Serial.println("BNO055 initialized");
}

// update sensor data
void SensorFB::update(){
    const unsigned long current_us = micros();
    float dt_s = (current_us - this->last_bno_update_us) * 1.0e-6f;
    this->last_bno_update_us = current_us;

    const imu::Vector<3> euler_raw =
        bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    const imu::Vector<3> acc_raw =
        bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    const imu::Vector<3> gyro_raw =
        bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    if (!this->bno_filter_initialized || dt_s <= 0.0f){
        // Initialize from the first measurement to avoid a zero-origin transient.
        this->euler = euler_raw;
        this->euler.x() = normalize_angle_deg(this->euler.x());
        this->euler.y() = normalize_angle_deg(this->euler.y());
        this->euler.z() = normalize_angle_deg(this->euler.z());
        this->euler_last = this->euler;
        this->acc = acc_raw;
        this->acc_last = this->acc;
        this->gyro = gyro_raw;
        this->bno_filter_initialized = true;
        return;
    }

    // Save the previous filtered values for derivative calculations.
    this->euler_last = this->euler;
    this->acc_last = this->acc;

    // angle_x is yaw. Keep it unfiltered, but express it in [-180, 180).
    this->euler.x() = normalize_angle_deg(euler_raw.x());
    this->euler.y() = low_pass_angle_deg(
        euler_raw.y(), this->euler_last.y(), ANGLE_LPF_CUTOFF_HZ, dt_s);
    this->euler.z() = low_pass_angle_deg(
        euler_raw.z(), this->euler_last.z(), ANGLE_LPF_CUTOFF_HZ, dt_s);

    this->acc.x() = low_pass_filter(
        acc_raw.x(), this->acc_last.x(), ACCEL_LPF_CUTOFF_HZ, dt_s);
    this->acc.y() = low_pass_filter(
        acc_raw.y(), this->acc_last.y(), ACCEL_LPF_CUTOFF_HZ, dt_s);
    this->acc.z() = low_pass_filter(
        acc_raw.z(), this->acc_last.z(), ACCEL_LPF_CUTOFF_HZ, dt_s);

    this->gyro.x() = low_pass_filter(
        gyro_raw.x(), this->gyro.x(), GYRO_LPF_CUTOFF_HZ, dt_s);
    this->gyro.y() = low_pass_filter(
        gyro_raw.y(), this->gyro.y(), GYRO_LPF_CUTOFF_HZ, dt_s);
    this->gyro.z() = low_pass_filter(
        gyro_raw.z(), this->gyro.z(), GYRO_LPF_CUTOFF_HZ, dt_s);
}

// getters
BNO055Data SensorFB::get_bno055_data() const{
    return {
        {
            static_cast<float>(this->acc.x()),
            static_cast<float>(this->acc.y()),
            static_cast<float>(this->acc.z())
        },
        {
            static_cast<float>(this->euler.x()),
            static_cast<float>(this->euler.y()),
            static_cast<float>(this->euler.z())
        },
        {
            static_cast<float>(this->gyro.x()),
            static_cast<float>(this->gyro.y()),
            static_cast<float>(this->gyro.z())
        }
    };
}

// state check
bool SensorFB::fall(){
    float fall_angle = 45.0f;
    if(this->euler.y() > -fall_angle && this->euler.y() < fall_angle && 
       this->euler.z() > -fall_angle && this->euler.z() < fall_angle){
        return false;
    }else{
        return true;
    }
}

// check body state
bool SensorFB::face_up(){
    if(-this->euler.y()<0){
        return true;
    }else{
        return false;
    }
}

// body angle feedback
float SensorFB::angle_phi_fb(){
    // rotate body base roll angle accordance with body angle.
    // calculate angle error
    float err = -this->euler.y() - PHI_TARGET_DEG;
    float err_last = -this->euler_last.y() - PHI_TARGET_DEG;
    float derr = err - err_last;

    err  = err  * PI / 180.0f;
    derr = derr * PI / 180.0f;

    float angle_phi_fb = this->kp_phi_body * err + this->kd_phi_body * derr;
    return angle_phi_fb;
}

// acceleration feedback (to phi)
int SensorFB::update_rate_fb(
    float t_ideal, array<float, 2> acc_ideal,
    array<float, 3> approx_coeff, float Tc, int update_rate, 
    float com_pos)
{
    // update last acc_ideal
    float acc = this->acc.y();
    float acc_last = this->acc_last.y();
    this->acc_ideal_last = acc_ideal[1];
    
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
        sig = 1;
    }else{
        sig = -1;
    }

    // estimate current pos and phase(time) based on current acceleration.
    float pos_y = acc * (Tc*Tc);
    float t_now;
    t_now = sqrt((pos_y - c_dash)/a) * sig;

    // error handling: if pos_y is out of range, set defalut value
    if(c_dash > 0 && pos_y < c_dash){
        pos_y = c_dash;
        t_now = t_ideal;
    }else if (c_dash < 0 && pos_y > c_dash){
        pos_y = c_dash;
        t_now = t_ideal;
    }
    this->last_pos_y = pos_y;

    float t_err = t_now - t_ideal;
    float t_derr = t_err - this->t_err_last;
    this->t_err_last = t_err;

    // float acc_fb = abs(this->kp_update_rate * t_err + this->kd_update_rate * t_derr) + 1.0f;
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
    this->last_update_rate_fb = update_rate_fb;

    // cast to int, and handle 0
    int update_rate_fb_int = (int)update_rate_fb;
    if (update_rate_fb_int == 0){
        update_rate_fb_int = 1;
    }
    return update_rate_fb_int;
}

// acceleration feedback (anterior and posterior direction)
array<float, 2> SensorFB::x0_vx0_fb(
    float tx, 
    float x0, float vx0, 
    float Tc, int control_step,
    float com_x_pos)
{
    // get acceleration, estimate position and velocity
    float acc = this->acc.x();
    // amplitude for front direction
    // if (acc > 0){
    //     acc *= 2.0f;
    // }
    float acc_last = this->acc_last.x();

    float pos_x      = acc      * (Tc*Tc);
    float pos_x_last = acc_last * (Tc*Tc);
    float vel_x = (pos_x - pos_x_last) / (1.0f / control_step);

    // single phase start calculation / estimation
    float Ct = cosh(tx/Tc);
    float St = sinh(tx/Tc);

    float Ct_sq_St_sq = Ct * Ct - St * St;
    float denom       = Ct_sq_St_sq;
    
    // ideal values at t1
    float x_t1_ideal  = x0 * Ct + Tc * vx0 * St;
    float vx_t1_ideal = x0 / Tc * St + vx0 * Ct;
    
    // feedback calculations
    float x0_fb_pos  =  x0 - (Ct * pos_x      - Tc * St * vx_t1_ideal) / denom;
    float vx0_fb_pos = vx0 - (-St/Tc * pos_x  + Ct * vx_t1_ideal)      / denom;
    
    float x0_fb_vel  =  x0 - (Ct * x_t1_ideal     - Tc * St * vel_x) / denom;
    float vx0_fb_vel = vx0 - (-St/Tc * x_t1_ideal + Ct * vel_x)      / denom;

    float x0_fb  = this->a_pos * x0_fb_pos  + this->a_vel * x0_fb_vel;
    float vx0_fb = this->a_pos * vx0_fb_pos + this->a_vel * vx0_fb_vel;

    float x0_fb_d  = x0_fb  - this->x0_fb_last;
    float vx0_fb_d = vx0_fb - this->vx0_fb_last;

    this->x0_fb_last  = x0_fb;
    this->vx0_fb_last = vx0_fb;

    float pd_x0_fb  = this->kp_x0_vx0 * x0_fb  + this->kd_x0_vx0 * x0_fb_d;
    float pd_vx0_fb = this->kp_x0_vx0 * vx0_fb + this->kd_x0_vx0 * vx0_fb_d;

    return {pd_x0_fb, pd_vx0_fb};
}
