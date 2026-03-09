#include "SensorFB.h"

static MotionSD* sd;
char f_update_rate[20];
char f_x0_vx0[20];

int i_update_rate = 0;
int i_x0_vx0 = 0;

float (*motions_update_rate)[18];
float (*motions_x0_vx0)[18];

SensorFB::SensorFB(){}

void SensorFB::init(MotionSD* s){
    motions_update_rate =
        (float (*)[18]) malloc(sizeof(float) * 600 * 18);

    motions_x0_vx0 =
        (float (*)[18]) malloc(sizeof(float) * 600 * 18);
    sd = s;
    int i = 0;
    while(true){
        sprintf(f_update_rate, "/update_rate_%d.csv", i);
        if(sd->is_file_exist(f_update_rate) == true){
            i++;
        }else{
            break;
        }
    }
    sprintf(f_x0_vx0, "/x0_vx0_%d.csv", i);

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
    this->euler.y() += this->phi * 180.0f / PI;
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

bool SensorFB::fly(){
    if (this->acc.z() - this->acc_last.z() < -5.0f){
        return true;
    }else{
        return false;
    }
}

bool SensorFB::hit_ground(){
    if (this->acc.z() - this->acc_last.z() > 5.0f){
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

    // convert angle to com position
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

    err  = err  * PI / 180.0f;
    derr = derr * PI / 180.0f;

    float angle_phi_fb = this->kp_phi_body * err + this->kd_phi_body * derr;
    return angle_phi_fb;
}

array<float, 3> SensorFB::vd_fb(array<float, 3> vd){
    array<float, 3> vd_fb = {
        kp_angle_vd * float(-this->euler.y()) + kd_angle_vd * float(-this->euler.y()),
        kp_angle_vd * float( this->euler.z()) + kd_angle_vd * float( this->euler.z()),
        0
    };
    return vd_fb;
}

// acceleration feedback
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
    // Serial.println();
    // Serial.print("a: "); Serial.print(a, 4); Serial.print(", b: "); Serial.print(b, 4); Serial.print(", c: "); Serial.println(c, 4);
    // Serial.print("acc_ideal: "); Serial.print(acc_ideal[1], 4); Serial.print(", acc: "); Serial.println(acc, 4);
    // Serial.print("ideal y: "); Serial.print(com_pos, 4); Serial.print(", calculated y: "); Serial.println(a * t_ideal * t_ideal + c_dash, 4);
    // Serial.print(", pos_y: "); Serial.println(pos_y, 4);
    // Serial.print("t_ideal: "); Serial.print(t_ideal, 4); Serial.print(", calculated t: "); Serial.println(sqrt((com_pos - c_dash)/a) * sig, 4);
    // Serial.print("t_now: "); Serial.println(sqrt((pos_y - c_dash)/a) * sig, 4);
    // Serial.print("t_err: "); Serial.println(t_err, 4);
    // Serial.print("update_rate_fb: "); Serial.println(update_rate_fb, 4);

    i_update_rate++;
    if (i_update_rate == 600){
        Serial.println("Writing update rate feedback data to SD card...");
        sd->write_long_motion(f_update_rate, motions_update_rate, 600);
    }else{
        float data[18] = {
            acc_ideal[1], acc, 0.0,
            com_pos, pos_y, 0.0,
            t_ideal, sqrt((pos_y - c_dash)/a) * sig, update_rate_fb, 0.0,
            0,0,0,0,0,0,0,0
        };

        memcpy(motions_update_rate[i_update_rate], data, sizeof(data));
    }
    return update_rate_fb_int;
}

array<float, 2> SensorFB::x0_vx0_fb(
    float tx, 
    float x0, float vx0, 
    float Tc, int control_step,
    float com_x_pos)
{
    // get acceleration, estimate position and velocity
    float acc      = this->acc.x();
    float acc_last = this->acc_last.x();

    float pos_x      = acc      * (Tc*Tc);
    float pos_x_last = acc_last * (Tc*Tc);
    float vel_x = (pos_x - pos_x_last) / (1.0f / control_step);

    // single phase start calculation / estimation
    float Ct = cosh(tx/Tc);
    float St = sinh(tx/Tc);

    float Ct_sq_St_sq = Ct * Ct - St * St;
    float denom       = tx * Ct_sq_St_sq;
    
    // ideal values at t1
    float x_t1_ideal  = x0 * Ct + Tc * vx0 * St;
    float vx_t1_ideal = (tx / Tc) * (x0 * St + Tc * vx0 * Ct);
    
    // feedback calculations
    float x0_fb_pos  =  x0 - (tx * Ct * pos_x     - Tc * St * vx_t1_ideal) / denom;
    float vx0_fb_pos = vx0 - (-tx/Tc * St * pos_x + Ct * vx_t1_ideal)      / denom;
    
    float x0_fb_vel  =  x0 - (tx * Ct * x_t1_ideal     - Tc * St * vel_x)  / denom;
    float vx0_fb_vel = vx0 - (-tx/Tc * St * x_t1_ideal + Ct * vel_x)       / denom;

    float x0_fb  = this->a_pos * x0_fb_pos  + this->a_vel * x0_fb_vel;
    float vx0_fb = this->a_pos * vx0_fb_pos + this->a_vel * vx0_fb_vel;

    float x0_fb_d  = x0_fb  - this->x0_fb_last;
    float vx0_fb_d = vx0_fb - this->vx0_fb_last;

    this->x0_fb_last  = x0_fb;
    this->vx0_fb_last = vx0_fb;

    float pd_x0_fb  = this->kp_x0_vx0 * x0_fb  + this->kd_x0_vx0 * x0_fb_d;
    float pd_vx0_fb = this->kp_x0_vx0 * vx0_fb + this->kd_x0_vx0 * vx0_fb_d;

    // Serial.println();
    // Serial.print("com_x_pos: "); Serial.println(com_x_pos, 4);
    // Serial.print("estimated x: "); Serial.print(pos_x, 4); Serial.print(", estimated vx: "); Serial.println(vel_x, 4);
    // Serial.print("x0: "); Serial.print(x0, 4); Serial.print(", vx0: "); Serial.println(vx0, 4);
    // Serial.print("x0_fb_pos: "); Serial.print(x0_fb_pos, 4); Serial.print(", vx0_fb_pos: "); Serial.println(vx0_fb_pos, 4);
    // Serial.print("x0_fb_vel: "); Serial.print(x0_fb_vel, 4); Serial.print(", vx0_fb_vel: "); Serial.println(vx0_fb_vel, 4);
    // Serial.print("x0_fb: "); Serial.print(x0_fb, 4); Serial.print(", vx0_fb: "); Serial.println(vx0_fb, 4);
    i_x0_vx0++;
    if (i_x0_vx0 == 600){
        sd->write_long_motion(f_x0_vx0, motions_x0_vx0, 600);
    }else{
        float data[18] = {
            acc, vel_x, 0.0,
            com_x_pos, pos_x, 0.0,
            x0, x0_fb_pos, x0_fb_vel,
            vx0, vx0_fb_pos, vx0_fb_vel,
            pd_x0_fb, pd_vx0_fb, 0.0
        };
        memcpy(motions_x0_vx0[i_x0_vx0], data, sizeof(data));
    }

    return {pd_x0_fb, pd_vx0_fb};
}
