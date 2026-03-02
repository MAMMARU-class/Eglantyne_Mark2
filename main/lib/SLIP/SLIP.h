#include <Arduino.h>
#include <cmath>

#define G 9.81 // m/s^2
#define M 1.2 // kg

using std::array;

class SLIP{
public:
    SLIP();
    // initialization
    void set_z0(float z0){ this->z0 = z0; }
    void set_z_flight(float z_flight){ this->z_flight = z_flight; }
    void set_foot_dist_y_base(float foot_dist_y_base){ this->foot_dist_y_base = foot_dist_y_base; }
    void set_foot_dist_x_max(float foot_dist_x_max){ this->foot_dist_x_max = foot_dist_x_max; }
    void set_T_sup_base(float T_sup_base){ this->T_sup_base = T_sup_base; }
    void set_T_sup_min(float T_sup_min){ this->T_sup_min = T_sup_min; }
    void set_fb_gain(float kxp, float kxd, float kyp, float kyd){
        this->kxp = kxp;
        this->kxd = kxd;
        this->kyp = kyp;
        this->kyd = kyd;
    }

    void calculate_initial_params();

    // basic state calculation
    float calc_basic_passing_com_vel(float p, float T_sup);
    float calc_basic_unpassing_com_vel(float p, float T_sup);
    float calc_basic_passing_foot_pos(float v, float T_sup);
    float calc_basic_unpassing_foot_pos(float v, float T_sup);

    // order normalization
    array<float, 3> normalize_vel(array<float, 3> v);

    // rotation
    array<float, 2> calc_rot_angle_limit(array<float, 2> cvn_last);
    array<float, 2> rotate_vec(array<float, 2> vec, float angle);

    // LIP calculation
    array<float, 2> calc_LIP_p(float t, array<float, 2> cp0, array<float, 2> cv0);
    array<float, 2> calc_LIP_v(float t, array<float, 2> cp0, array<float, 2> cv0);
    void calc_approx_coeff(array<float, 2> cp0, array<float, 2> cv0, float T_sup);

    // foot pos pd
    array<float, 2> foot_pos_pd(array<float, 2> cvn_last, array<float, 2> cvn_m1_last, array<float, 3> vd, array<float, 2> pn, float T_sup);

    // double support spline
    void calc_double_support_coeff(float T_ds, array<float, 2> cpn_d_0, array<float, 2> cvn_d_0, array<float, 2> can_d_0, array<float, 2> cpn_d_T, array<float, 2> cvn_d_T, array<float, 2> can_d_T);
    array<float, 2> calc_double_support_com_p(float t);
    array<float, 2> calc_double_support_com_a(float t);

    // z
    array<float, 2>calc_com_z(float t, float T_sup, float ds_ratio);

    // gettera
    float get_z0(){ return this->z0; }
    float get_Tc(){ return this->Tc; }
    float get_foot_dist_y_base(){ return this->foot_dist_y_base; }
    float get_T_sup_base(){ return this->T_sup_base; }
    array<float, 3> get_approx_coeff(){ return this->approx_coeff; }

private:
    // params
    float z0 = 158*0.001;
    float z_flight = 0.04;
    float foot_dist_y_base = 0.03;
    float foot_dist_x_max = 0.12;

    float T_sup_base = 0.6;
    float T_sup_min = 0.3;

    float kxp = 0.003;
    float kxd = 0.0003;
    float kyp = 0.003;
    float kyd = 0.0003;

    // calculated params
    float Tc;
    float v_mag_boarder;
    float v_mag_max;

    array<array<float, 6>, 2> ds_coeff;

    // approximation coeff for y
    array<float, 3> approx_coeff;
};
