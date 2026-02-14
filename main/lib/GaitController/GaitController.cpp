#include "GaitController.h"
#include <cmath>

GaitController::GaitController(){}

void GaitController::init_param_walk(){
    model.set_z0(0.158f);
    model.set_z_flight(0.03f);
    model.set_foot_dist_y_base(0.03f);
    model.set_foot_dist_x_max(0.12f);
    model.set_T_sup_base(0.6f);
    model.set_T_sup_min(0.3f);
    model.set_fb_gain(0.003f, 0.0003f, 0.003f, 0.0003f);
    model.calculate_initial_params();

    this->pivot = Pivot::LEFT;
    this->cpn_start = {0.0f, this->pivot_sign(this->pivot) * model.get_foot_dist_y_base()};

    this->pn = {-1*this->cpn_start[0], -1*this->cpn_start[1]};
    this->pn_p1 = {0.0f, this->pivot_sign(this->pivot) * model.get_foot_dist_y_base()};
    this->p_n2p1 = {0.0f, -2 * this->pivot_sign(this->pivot) * model.get_foot_dist_y_base()};

    this->T_sup = model.get_T_sup_base();
    this->T_sup_last = 0.0f;

    this->cvn_m1_start = {0.0f, model.calc_basic_unpassing_com_vel(-this->pn[1], this->T_sup)};
    this->cvn_start = model.calc_LIP_v(this->T_sup, {-this->pn[0], -this->pn[1]}, this->cvn_m1_start);

    this->ds_ratio = 0.2f;
    this->body_angle = 0.0f;
}

void GaitController::init_param_fight(){
}

void GaitController::inverse_pivot(){
    if(this->pivot == Pivot::RIGHT){
        this->pivot = Pivot::LEFT;
    }
    else{
        this->pivot = Pivot::RIGHT;
    }
}

int GaitController::pivot_sign(Pivot p){
    return (p == Pivot::RIGHT) ? 1 : -1;
}

/* -------------------------
update state variables
---------------------------- */
void GaitController::init_state_variables()
{
    float Tc = model.get_Tc();

    array<float,2> cpn_d_0, cvn_d_0, can_d_0;
    array<float,2> cpn_d_T, cvn_d_T, can_d_T;

    this->cvn_m1_start = this->cvn_start;

    // last sprin at single -> double 
    cpn_d_0 =
        model.calc_LIP_p(
            this->T_sup * (1.0f - this->ds_ratio * 0.5f),
            {-this->pn[0], -this->pn[1]},
            this->cvn_start
        );

    array<float,2> cpn_last =
        model.calc_LIP_p(
            this->T_sup,
            {-this->pn[0], -this->pn[1]},
            this->cvn_start
        );
    cpn_d_0 = {cpn_d_0[0]-cpn_last[0], cpn_d_0[1]-cpn_last[1]};

    cvn_d_0 =
        model.calc_LIP_v(
            this->T_sup * (1.0f - this->ds_ratio * 0.5f),
            {-this->pn[0], -this->pn[1]},
            this->cvn_start
        );

    can_d_0 = {cpn_d_0[0]/(Tc*Tc), cpn_d_0[1]/(Tc*Tc)};

    // calculate variables for next sprine
    // calculate cn_start
    this->cvn_start = this->cvn_last;
    
    // update foot positions
    this->pn = this->pn_p1;

    this->p_n2m1 = {-this->p_n2p1[0], -this->p_n2p1[1]};
    this->p_n2m1 = model.rotate_vec(this->p_n2m1, -this->body_angle);

    // update T
    this->T_sup_last = this->T_sup;
    this->T_sup = this->T_sup_next;

    // rotate and convert cn_d_0
    cpn_d_0 = model.rotate_vec(cpn_d_0, -this->body_angle);
    cpn_d_0 = {cpn_d_0[0]-this->pn[0], cpn_d_0[1]-this->pn[1]};
    cvn_d_0 = model.rotate_vec(cvn_d_0, -this->body_angle);
    can_d_0 = model.rotate_vec(can_d_0, -this->body_angle);

    // calculate cn_d_T
    cpn_d_T =
        model.calc_LIP_p(
            this->T_sup * this->ds_ratio * 0.5f,
            {-this->pn[0], -this->pn[1]},
            this->cvn_start
        );

    cvn_d_T =
        model.calc_LIP_v(
            this->T_sup * this->ds_ratio * 0.5f,
            {-this->pn[0], -this->pn[1]},
            this->cvn_start
        );

    can_d_T = {cpn_d_T[0]/(Tc*Tc), cpn_d_T[1]/(Tc*Tc)};

    // update ds_coeff
    model.calc_double_support_coeff(
        this->T_ds,
        cpn_d_0, cvn_d_0, can_d_0,
        cpn_d_T, cvn_d_T, can_d_T
    );
}

void GaitController::update_state_variables(array<float, 3> vd){
    // normalize control input
    vd = model.normalize_vel(vd);
    float v_mag = sqrt(vd[0]*vd[0] + vd[1]*vd[1]);
    this->T_sup_next = model.calc_T_sup(v_mag);

    // decide angle
    array<float, 2> angle_limits = model.calc_rot_angle_limit(this->cvn_start);
    float rotation = vd[2];
    if (rotation > 0) {
        rotation = angle_limits[0] * abs(rotation);
    } else {
        rotation = angle_limits[1] * abs(rotation);
    }
    this->body_angle = rotation;

    // decide pn_p1
    this->cvn_last = model.calc_LIP_v(this->T_sup, {-this->pn[0], -this->pn[1]}, this->cvn_start);
    this->cvn_last = model.rotate_vec(this->cvn_last, -this->body_angle);
    this->pn_p1 = model.foot_pos_pd(this->cvn_last, this->cvn_start, vd, this->pn, this->T_sup_next);

    array<float, 2> cpn_last = model.calc_LIP_p(this->T_sup, {-this->pn[0], -this->pn[1]}, this->cvn_start);
    array<float, 2> pn_p1_rot = model.rotate_vec(this->pn_p1, this->body_angle);
    this->p_n2p1 = {
        cpn_last[0]+pn_p1_rot[0],
        cpn_last[1]+pn_p1_rot[1]
    };

    // calculate T_ds
    this->T_ds = (this->T_sup_next + this->T_sup) / 2 * this->ds_ratio;
}

/* -------------------------
init gait trajectory param
---------------------------- */
void GaitController::init_start(){
    // com state at start
    this->inverse_pivot();
    this->T_sup = model.get_T_sup_base();

    // initialize foot positions
    this->pn = this->pn_p1;
    this->p_n2m1 = {-1*this->p_n2p1[0], -1*this->p_n2p1[1]};
    this->p_n2m1 = model.rotate_vec(this->p_n2m1, this->body_angle);
    Serial.print("pn: "); Serial.print(this->pn[0]); Serial.print(", "); Serial.println(this->pn[1]);

    // calculate state variables
    Serial.print("cvn_satrt: "); Serial.print(this->cvn_start[0]); Serial.print(", "); Serial.println(this->cvn_start[1]);
    array<float, 2> cpn_d_0 = {-this->pn[0], -this->pn[1]};
    array<float, 2> cvn_d_0 = {0.0f, 0.0f};
    array<float, 2> can_d_0 = {0.0f, 0.0f};

    array<float, 2> cpn_d_T =
        model.calc_LIP_p(
            this->T_sup * this->ds_ratio * 0.5f,
            {-this->pn[0], -this->pn[1]},
            this->cvn_start
        );
    array<float, 2> cvn_d_T =
        model.calc_LIP_v(
            this->T_sup * this->ds_ratio * 0.5f,
            {-this->pn[0], -this->pn[1]},
            this->cvn_start
        );
    float Tc = model.get_Tc();
    array<float, 2> can_d_T = {
        cpn_d_T[0] / (Tc * Tc),
        cpn_d_T[1] / (Tc * Tc)
    };
    this->T_ds = this->T_sup * this->ds_ratio * 0.5f;

    this->p_n2m1 = {
        -this->p_n2m1[0],
        -this->p_n2m1[1]
    };

    model.calc_double_support_coeff(
            this->T_ds,
            cpn_d_0,
            cvn_d_0,
            can_d_0,
            cpn_d_T,
            cvn_d_T,
            can_d_T
        );
}

void GaitController::init_end(){
}

void GaitController::init_single_0(){
    this->pivot_leg_angle = 0.0;
    this->swing_leg_angle = this->body_angle;
    this->single_start_com = this->model.calc_LIP_p(
        this->T_sup*this->ds_ratio*0.5f,
        {-this->pn[0], -this->pn[1]},
        this->cvn_start
    );
    swing_com_0 = {
        -this->p_n2m1[0] + this->single_start_com[0],
        -this->p_n2m1[1] + this->single_start_com[1]
    };
    swing_com_0 = model.rotate_vec(swing_com_0, this->body_angle);

    array<float, 2> pivot_com_half = model.calc_LIP_p(T_sup*0.5f, {-pn[0], -pn[1]}, this->cvn_start);
    float pivot_com_diff_y = pivot_com_half[1] - this->single_start_com[1]
    swing_com_half = {0.0f, swing_com_0[1] + pivot_com_half};
}
void GaitController::init_single_half(){
    array<float, 2> single_last_com = this->model.calc_LIP_p(
        this->T_sup*(1.0f - this->ds_ratio*0.5f),
        {-this->pn[0], -this->pn[1]},
        this->cvn_start
    );
    this->swing_com_last = {
        -this->p_n2p1[0] + single_last_com[0],
        -this->p_n2p1[1] + single_last_com[1]
    };
    this->swing_com_last = model.rotate_vec(this->swing_com_last, -this->body_angle);
    this->swing_leg_angle = 0.0f;
}

/* -------------------------
calculate trajectory for each steps
---------------------------- */
array<array<float, 5>, 2> GaitController::calc_com_traj_single(float t){
    float T_ss = this->T_sup * (1.0f - this->ds_ratio);

    array<float, 2> com = model.calc_LIP_p(t + this->T_sup*this->ds_ratio/2, {-this->pn[0], -this->pn[1]}, this->cvn_start);
    array<float, 2> com_z = model.calc_com_z(t, this->T_sup, this->ds_ratio);
    array<float, 2> swing_com;

    if (t < T_ss/2){
        swing_com[0] = this->swing_com_0[0] * (T_ss*0.5f - t) + this->swing_com_half[0] * t;
        swing_com[1] = this->swing_com_0[1] * (T_ss*0.5f - t) + this->swing_com_half[1] * t;
        swing_com = {swing_com[0]/(T_ss*0.5f), swing_com[1]/(T_ss*0.5f)};
        this->swing_leg_angle = this->body_angle * (T_ss*0.5f - t) / (T_ss*0.5f);
    }else{
        swing_com[0] = this->swing_com_half[0] * (T_ss - t) + this->swing_com_last[0] * (t - T_ss*0.5f);
        swing_com[1] = this->swing_com_half[1] * (T_ss - t) + this->swing_com_last[1] * (t - T_ss*0.5f);
        swing_com = {swing_com[0]/(T_ss - T_ss*0.5f), swing_com[1]/(T_ss - T_ss*0.5f)};
        this->pivot_leg_angle = this->body_angle * (t - T_ss*0.5f) / (T_ss - T_ss*0.5f);
    }
    array<float, 5> com_pivot = {com[0], com[1], com_z[0], this->pivot_leg_angle, 0};
    array<float, 5> com_else = {swing_com[0], swing_com[1], com_z[1], this->swing_leg_angle, 0};

    if(pivot == Pivot::RIGHT){
        return {com_pivot, com_else};
    }else{
        return {com_else, com_pivot};
    }
}

array<array<float, 5>, 2> GaitController::calc_com_traj_double(float t){
    array<float, 2> com = model.calc_double_support_com_p(t);
    float com_z = model.get_z0();

    array<float, 5> com_pivot = {com[0], com[1], com_z, 0, 0};
    array<float, 2> com_else_xy = {com[0]-p_n2m1[0], com[1]-p_n2m1[1]};
    com_else_xy = model.rotate_vec(com_else_xy, this->body_angle);
    array<float, 5> com_else = {com_else_xy[0], com_else_xy[1], com_z, this->body_angle, 0};

    if(pivot == Pivot::RIGHT){
        return {com_pivot, com_else};
    }else{
        return {com_else, com_pivot};
    }
}
