#include "GaitController.h"
#include <cmath>

GaitController::GaitController(){}

/* #########################################################################
CALCULATION PARAMETERS
##########################################################################*/
void GaitController::init_param_walk(float z0){
    // set max order input
    this->set_vd_max_abs({0.3f, 0.05f, 1.0f});
    // initialize control parameters
    model.set_z0(z0);
    model.set_z_flight(0.01f);
    model.set_foot_dist_y_base(0.04f);
    model.set_foot_dist_x_max(0.12f);
    model.set_T_sup_base(0.2f);
    model.set_T_sup_min(0.3f);
    // model.set_fb_gain(0.003, 0.0003f, 0.003, 0.0003);
    model.set_fb_gain(0.00001, 0.000001f, 0.00001, 0.000001);
    model.calculate_initial_params();
    set_ds_ratio(0.3f);
}

void GaitController::init_pose_walk(){
    // set initial pose
    this->pivot = Pivot::LEFT;
    this->cpn_start = {0.0f, this->pivot_sign(this->pivot) * model.get_foot_dist_y_base()};

    this->pn = {-1*this->cpn_start[0], -1*this->cpn_start[1]};
    this->pn_p1 = {0.0f, this->pivot_sign(this->pivot) * model.get_foot_dist_y_base()};
    this->p_n2p1 = {0.0f, -2 * this->pivot_sign(this->pivot) * model.get_foot_dist_y_base()};

    this->T_sup = model.get_T_sup_base();
    this->T_sup_last = 0.0f;

    this->cvn_m1_start = {0.0f, model.calc_basic_unpassing_com_vel(-this->pn[1], this->T_sup)};
    this->cvn_start = model.calc_LIP_v(this->T_sup, {-this->pn[0], -this->pn[1]}, this->cvn_m1_start);

    this->body_angle = 0.0f;
}

void GaitController::init_param_fight(float z0){
    // set max order input
    this->set_vd_max_abs({0.2f, 0.2f, 1.0f});
    // initialize control parameters
    model.set_z0(z0);
    model.set_z_flight(0.01f);
    model.set_foot_dist_y_base(0.06f);
    model.set_foot_dist_x_max(0.12f);
    model.set_T_sup_base(0.2f);
    model.set_T_sup_min(0.3f);
    // model.set_fb_gain(0.003, 0.0003f, 0.003, 0.0003);
    model.set_fb_gain(0.00001, 0.000001f, 0.00001, 0.000001);
    model.calculate_initial_params();
    set_ds_ratio(0.2f);
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

/* #########################################################################
STATE VARIABLES
##########################################################################*/
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

void GaitController::update_state_variables(array<float, 3> vd, array<float, 2> foot_pos_fb, float body_angle_order){
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
    this->body_angle = rotation + body_angle_order;

    // decide pn_p1
    this->cvn_last = model.calc_LIP_v(this->T_sup, {-this->pn[0], -this->pn[1]}, this->cvn_start);
    this->cvn_last = model.rotate_vec(this->cvn_last, -this->body_angle);
    this->pn_p1 = model.foot_pos_pd(this->cvn_last, this->cvn_start, vd, this->pn, this->T_sup_next);
    this->pn_p1[0] += foot_pos_fb[0];
    this->pn_p1[1] += foot_pos_fb[1];

    array<float, 2> cpn_last = model.calc_LIP_p(this->T_sup, {-this->pn[0], -this->pn[1]}, this->cvn_start);
    array<float, 2> pn_p1_rot = model.rotate_vec(this->pn_p1, this->body_angle);
    this->p_n2p1 = {
        cpn_last[0]+pn_p1_rot[0],
        cpn_last[1]+pn_p1_rot[1]
    };

    // calculate T_ds
    this->T_ds = (this->T_sup_next + this->T_sup) / 2 * this->ds_ratio;
}

/* #########################################################################
INITIALIZE PHASE PARAMETERS
at the beginning of each phase (or middle for single support), initialize the parameters related to the phase transition
- start            : Similar to "init_state_variables", except setting initial com velocity and acceleration to 0.
- end              : Similar to "init_state_variables", except setting last com velocity and acceleration to 0.
- single (at 0)    : 
- single (at half) : Calculate swing foot goal position and update swing leg angle
##########################################################################*/
void GaitController::init_start(){
    float Tc = model.get_Tc();

    array<float,2> cpn_d_0, cvn_d_0, can_d_0;
    array<float,2> cpn_d_T, cvn_d_T, can_d_T;

    // com state at start
    this->T_sup = model.get_T_sup_base();

    // initialize foot positions
    this->pn = this->pn_p1;
    this->p_n2m1 = {-1*this->p_n2p1[0], -1*this->p_n2p1[1]};
    this->p_n2m1 = model.rotate_vec(this->p_n2m1, this->body_angle);

    // calculate state variables
    cpn_d_0 = {-this->pn[0], -this->pn[1]};
    cvn_d_0 = {0.0f, 0.0f};
    can_d_0 = {0.0f, 0.0f};

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
    can_d_T = {
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
    float Tc = model.get_Tc();

    array<float,2> cpn_d_0, cvn_d_0, can_d_0;
    array<float,2> cpn_d_T, cvn_d_T, can_d_T;

    // com state at start
    this->T_sup = model.get_T_sup_base();

    // initialize foot positions
    this->pn = this->pn_p1;
    this->p_n2m1 = {-1*this->p_n2p1[0], -1*this->p_n2p1[1]};
    this->p_n2m1 = model.rotate_vec(this->p_n2m1, this->body_angle);

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

    cpn_d_T = {0.0f, model.get_foot_dist_y_base()*this->pivot_sign(this->pivot)};
    cvn_d_T = {0.0f, 0.0f};
    can_d_T = {0.0f, 0.0f};

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
    float pivot_com_diff_y = pivot_com_half[1] - this->single_start_com[1];
    swing_com_half = {0.0f, swing_com_0[1] + pivot_com_diff_y};
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

/* #########################################################################
TRAJECTORY CALCULATION
calculate trajectory for each steps
##########################################################################*/
array<array<float, 5>, 3> GaitController::calc_com_traj_single(float t){
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

    // calculate com acceleration
    float Tc = model.get_Tc();
    array<float, 5> com_acc = {com[0]/(Tc*Tc), com[1]/(Tc*Tc), 0.0f, 0.0f, 0.0f};

    if(pivot == Pivot::RIGHT){
        return {com_pivot, com_else, com_acc};
    }else{
        return {com_else, com_pivot, com_acc};
    }
}

array<array<float, 5>, 3> GaitController::calc_com_traj_double(float t){
    array<float, 2> com = model.calc_double_support_com_p(t);
    float com_z = model.get_z0();

    array<float, 5> com_pivot = {com[0], com[1], com_z, 0, 0};
    array<float, 2> com_else_xy = {com[0]-p_n2m1[0], com[1]-p_n2m1[1]};
    com_else_xy = model.rotate_vec(com_else_xy, this->body_angle);
    array<float, 5> com_else = {com_else_xy[0], com_else_xy[1], com_z, this->body_angle, 0};

    // calculate com acceleration
    array<float, 2> com_acc = model.calc_double_support_com_a(t);
    array<float, 5> com_acc_5d = {com_acc[0], com_acc[1], 0.0f, 0.0f, 0.0f};

    if(pivot == Pivot::RIGHT){
        return {com_pivot, com_else, com_acc_5d};
    }else{
        return {com_else, com_pivot, com_acc_5d};
    }
}
