#include "SLIP.h"
#include <cmath>

SLIP::SLIP(){}

/* -----------------------------
 * initial params
 * ----------------------------- */
void SLIP::calculate_initial_params(){
    this->Tc = std::sqrt(z0 / G);
    this->v_mag_boarder = calc_basic_passing_com_vel(-1 * foot_dist_x_max, T_sup_base);
    this->v_mag_max     = calc_basic_passing_com_vel(-1 * foot_dist_x_max, T_sup_min);
}

/* -----------------------------
 * basic state calculation
 * ----------------------------- */
float SLIP::calc_basic_passing_com_vel(float p, float T_sup){
    return (p / Tc) * ( std::sinh(T_sup / Tc) / (1.0f - std::cosh(T_sup / Tc)) );
}

float SLIP::calc_basic_unpassing_com_vel(float p, float T_sup){
    return (p / Tc) * ( std::sinh(T_sup / Tc) / (-1.0f - std::cosh(T_sup / Tc)) );
}

float SLIP::calc_basic_passing_foot_pos(float v, float T_sup){
    return Tc * v * (std::cosh(T_sup / Tc) - 1.0f) / std::sinh(T_sup / Tc);
}

float SLIP::calc_basic_unpassing_foot_pos(float v, float T_sup){
    return Tc * v * (std::cosh(T_sup / Tc) + 1.0f) / std::sinh(T_sup / Tc);
}

/* -----------------------------
 * order normalization
 * ----------------------------- */
array<float, 3> SLIP::normalize_vel(array<float, 3> v){
    array<float, 3> out = {v[0], v[1], v[2]};

    float v_mag = std::sqrt(v[0]*v[0] + v[1]*v[1]);
    if (v_mag > v_mag_max){
        out[0] = v_mag_max * v[0] / v_mag;
        out[1] = v_mag_max * v[1] / v_mag;
        out[2] = v[2];
    }
    return out;
}

float SLIP::calc_T_sup(float v_mag){
    return T_sup_base;
    // if (v_mag == 0.0f){
    //     return 0.0f;
    // } else if (v_mag < v_mag_boarder){
    //     return T_sup_base;
    // } else {
    //     return T_sup_base
    //          - T_sup_min * ((v_mag - v_mag_boarder) / (v_mag_max - v_mag_boarder));
    // }
}

/* -----------------------------
 * rotation
 * ----------------------------- */
array<float, 2> SLIP::calc_rot_angle_limit(array<float, 2> cvn_last){
    if (std::abs(cvn_last[0]) < 0.005f || std::abs(cvn_last[1]) < 0.005f){
        return {0.0f, 0.0f};
    }

    float a1 = std::atan2(cvn_last[1], cvn_last[0]);
    float y_v_lim = foot_dist_y_base / Tc;
    float cv_mag = std::sqrt(cvn_last[0]*cvn_last[0] + cvn_last[1]*cvn_last[1]);

    float a2;
    if (cv_mag < y_v_lim){
        a2 = 0.5f;
    } else {
        a2 = std::acos(y_v_lim / cv_mag);
    }
    a2 *= -1.0f * (a1 / std::abs(a1));

    a1 *= 0.5f;
    a2 *= 0.5f;

    if (a1 > a2){
        return {a1, a2};
    } else {
        return {a2, a1};
    }
}

array<float, 2> SLIP::rotate_vec(array<float, 2> vec, float angle){
    float c = std::cos(angle);
    float s = std::sin(angle);

    return {
        c * vec[0] - s * vec[1],
        s * vec[0] + c * vec[1]
    };
}

/* -----------------------------
 * LIP calculation
 * ----------------------------- */
array<float, 2> SLIP::calc_LIP_p(float t, array<float, 2> cp0, array<float, 2> cv0){
    float x = cp0[0] * std::cosh(t / Tc) + Tc * cv0[0] * std::sinh(t / Tc);
    float y = cp0[1] * std::cosh(t / Tc) + Tc * cv0[1] * std::sinh(t / Tc);
    return {x, y};
}

array<float, 2> SLIP::calc_LIP_v(float t, array<float, 2> cp0, array<float, 2> cv0){
    float vx = cp0[0] / Tc * std::sinh(t / Tc) + cv0[0] * std::cosh(t / Tc);
    float vy = cp0[1] / Tc * std::sinh(t / Tc) + cv0[1] * std::cosh(t / Tc);
    return {vx, vy};
}

void SLIP::calc_approx_coeff(array<float, 2> cp0, array<float, 2> cv0){
    this->approx_coeff[0] = 1/2.0f / (Tc*Tc) * cp0[1];
    this->approx_coeff[1] = 1 / (Tc*Tc) * cv0[1];
    this->approx_coeff[2] = cp0[1];
}

/* -----------------------------
 * foot position PD
 * ----------------------------- */
array<float, 2> SLIP::foot_pos_pd(array<float, 2> cvn_last,
                                 array<float, 2> cvn_m1_last,
                                 array<float, 3> vd,
                                 array<float, 2> pn,
                                 float T_sup)
{
    // somehow doesnt work if vd[0] == 0;
    if (vd[0] == 0.0f){
        vd[0] = 1e-4f;
    }

    array<float, 2> err;
    err[0] = vd[0] - cvn_last[0];

    float basic_foot_y = foot_dist_y_base * pn[1] / std::abs(pn[1]);
    float vd_y_sum = -1 * calc_basic_unpassing_com_vel(basic_foot_y, T_sup) + vd[1];
    err[1] = vd_y_sum + cvn_last[1];

    array<float, 2> derr;
    derr[0] = -cvn_last[0] + cvn_m1_last[0];
    derr[1] = -cvn_last[1] - cvn_m1_last[1];

    array<float, 2> p_base;
    p_base[0] = calc_basic_passing_foot_pos(cvn_last[0], T_sup);
    p_base[1] = calc_basic_unpassing_foot_pos(cvn_last[1], T_sup);

    float px = p_base[0] - kxp * err[0] - kxd * derr[0];
    float py = p_base[1] - kyp * err[1] - kyd * derr[1];

    return {px, py};
}

/* -----------------------------
 * double support spline
 * ----------------------------- */
void SLIP::calc_double_support_coeff(float T_ds,
                                array<float, 2> cpn_d_0,
                                array<float, 2> cvn_d_0,
                                array<float, 2> can_d_0,
                                array<float, 2> cpn_d_T,
                                array<float, 2> cvn_d_T,
                                array<float, 2> can_d_T)
{
    array<float, 6> ax, ay;

    ax[0] = cpn_d_0[0];
    ax[1] = cvn_d_0[0];
    ax[2] = 0.5f * can_d_0[0];
    ax[3] = (1.0f/(2*T_ds*T_ds*T_ds)) *
            (20*(cpn_d_T[0]-cpn_d_0[0])
            -(8*cvn_d_T[0]+12*cvn_d_0[0])*T_ds
            -(3*can_d_0[0]-can_d_T[0])*T_ds*T_ds);
    ax[4] = (1.0f/(2*T_ds*T_ds*T_ds*T_ds)) *
            (30*(cpn_d_0[0]-cpn_d_T[0])
            +(14*cvn_d_T[0]+16*cvn_d_0[0])*T_ds
            +(3*can_d_0[0]-2*can_d_T[0])*T_ds*T_ds);
    ax[5] = (1.0f/(2*T_ds*T_ds*T_ds*T_ds*T_ds)) *
            (12*(cpn_d_T[0]-cpn_d_0[0])
            -6*(cvn_d_T[0]+cvn_d_0[0])*T_ds
            -(can_d_0[0]-can_d_T[0])*T_ds*T_ds);

    ay[0] = cpn_d_0[1];
    ay[1] = cvn_d_0[1];
    ay[2] = 0.5f * can_d_0[1];
    ay[3] = (1.0f/(2*T_ds*T_ds*T_ds)) *
            (20*(cpn_d_T[1]-cpn_d_0[1])
            -(8*cvn_d_T[1]+12*cvn_d_0[1])*T_ds
            -(3*can_d_0[1]-can_d_T[1])*T_ds*T_ds);
    ay[4] = (1.0f/(2*T_ds*T_ds*T_ds*T_ds)) *
            (30*(cpn_d_0[1]-cpn_d_T[1])
            +(14*cvn_d_T[1]+16*cvn_d_0[1])*T_ds
            +(3*can_d_0[1]-2*can_d_T[1])*T_ds*T_ds);
    ay[5] = (1.0f/(2*T_ds*T_ds*T_ds*T_ds*T_ds)) *
            (12*(cpn_d_T[1]-cpn_d_0[1])
            -6*(cvn_d_T[1]+cvn_d_0[1])*T_ds
            -(can_d_0[1]-can_d_T[1])*T_ds*T_ds);

    this->ds_coeff = {ax, ay};
}

array<float, 2> SLIP::calc_double_support_com_p(float t){
    float x = 0.0f, y = 0.0f;
    float tt = 1.0f;

    for (int i = 0; i < 6; i++){
        x += this->ds_coeff[0][i] * tt;
        y += this->ds_coeff[1][i] * tt;
        tt *= t;
    }
    return {x, y};
}
array<float, 2> SLIP::calc_double_support_com_a(float t){
    float ax = 0.0f, ay = 0.0f;
    float tt = 1.0f;

    for (int i = 2; i < 6; i++){
        ax += this->ds_coeff[0][i] * tt * i * (i-1);
        ay += this->ds_coeff[1][i] * tt * i * (i-1);
        tt *= t;
    }
    return {ax, ay};
}

/* -----------------------------
 * COM Z trajectory
 * ----------------------------- */
array<float, 2> SLIP::calc_com_z(float t, float T_sup, float ds_ratio){
    float T_ss = T_sup * (1.0f - ds_ratio);

    // float z = this->z0 + std::cos(2*M_PI*t/T_ss)*T_sup/40.0f - T_sup/40.0f;
    float z = this->z0 + std::cos(2*M_PI*t/T_ss)*0.3/100.0f - 0.3/100.0f;

    float zh;
    // if (t < T_ss/2){
    //     zh = std::sin(M_PI*t/T_ss) * this->z_flight;
    // } else {
    //     zh = std::cos(2*M_PI*(t - T_ss/2)/T_ss) * this->z_flight/2.0f + this->z_flight/2.0f;
    // }
    zh = std::sin(M_PI*t/T_ss) * this->z_flight;

    return {z, z - zh};
}
