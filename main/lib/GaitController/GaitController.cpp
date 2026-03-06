#include "GaitController.h"
#include <cmath>

GaitController::GaitController(){}

/* #########################################################################
CALCULATION PARAMETERS
##########################################################################*/
void GaitController::init_param_walk(float z0){
    // set max order input
    // this->set_vd_max_abs({0.09f, 0.1f, 0.6f});
    this->set_vd_max_abs({0.00001f, 0.1f, 0.6f});
    // initialize control parameters
    model.set_z0(z0);
    // model.set_z_flight(0.025f);
    model.set_z_flight(0.035f);
    model.set_foot_dist_y_base(0.045f);
    model.set_foot_dist_x_max(0.12f);
    model.set_T_sup_base(0.15f);
    // model.set_T_sup_base(0.18f);
    model.set_T_sup_min(0.3f);
    model.set_fb_gain(0.03, 0.003f, 0.05f, 0.005f);
    model.calculate_initial_params();
    set_ds_ratio(0.35f);

    this->T_sup = model.get_T_sup_base();
    this->T_ds = this->T_sup * this->ds_ratio;
}

void GaitController::init_param_crouch(float z0){
    // set max order input
    // this->set_vd_max_abs({0.09f, 0.1f, 0.6f});
    this->set_vd_max_abs({0.1f, 0.1f, 0.6f});
    // initialize control parameters
    model.set_z0(z0);
    // model.set_z_flight(0.025f);
    model.set_z_flight(0.015f);
    model.set_foot_dist_y_base(0.03f);
    model.set_foot_dist_x_max(0.12f);
    model.set_T_sup_base(0.15f);
    // model.set_T_sup_base(0.18f);
    model.set_T_sup_min(0.3f);
    model.set_fb_gain(0.01, 0.001f, 0.01f, 0.001f);
    model.calculate_initial_params();
    set_ds_ratio(0.35f);

    this->T_sup = model.get_T_sup_base();
    this->T_ds = this->T_sup * this->ds_ratio;
}

void GaitController::init_param_fight(float z0){
    // set max order input
    this->set_vd_max_abs({0.01f, 1.5f, 0.1f});
    // initialize control parameters
    model.set_z0(z0);
    model.set_z_flight(0.001f);
    model.set_foot_dist_y_base(0.06f);
    model.set_foot_dist_x_max(0.12f);
    model.set_T_sup_base(0.2f);
    model.set_T_sup_min(0.3f);
    // model.set_fb_gain(0.003, 0.0003f, 0.003, 0.0003);
    model.set_fb_gain(0.01, 0.001f, 0.01, 0.001);
    model.calculate_initial_params();
    set_ds_ratio(0.2f);

    this->T_sup = model.get_T_sup_base();
    this->T_ds = this->T_sup * this->ds_ratio;
}

void GaitController::init_pose(){
    // set initial pose
    this->pivot = Pivot::LEFT;
    this->cpn_start = {0.0f, this->pivot_sign(this->pivot) * model.get_foot_dist_y_base()};

    this->pn = {-1*this->cpn_start[0], -1*this->cpn_start[1]};
    this->pn_p1 = {0.0f, this->pivot_sign(this->pivot) * model.get_foot_dist_y_base()};
    this->p_n2p1 = {0.0f, -2 * this->pivot_sign(this->pivot) * model.get_foot_dist_y_base()};

    this->T_sup = model.get_T_sup_base();

    array<float, 2> cvn_start_local = {0.0f, model.calc_basic_unpassing_com_vel(-this->pn[1], this->T_sup)};
    this->cvn_last = model.calc_LIP_v(
        this->T_sup,
        this->T_sup,
        {-this->pn[0], -this->pn[1]}, 
        cvn_start_local
    );

    this->body_angle = 0.0f;

    this->p_n2m1 = {-1*this->p_n2p1[0], -1*this->p_n2p1[1]};
    this->p_n2p1 = this->p_n2m1;
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
void GaitController::init_state_variables(bool zero_start, bool zero_end)
{
    // initialize T_sup_x
    this->T_sup_x = this->T_sup;
    this->T_ds = this->T_sup * this->ds_ratio;

    // calculate cn_d_0 and cn_d_T for double support sprine
    float Tc = model.get_Tc();

    array<float,2> cpn_d_0, cvn_d_0, can_d_0;
    array<float,2> cpn_d_T, cvn_d_T, can_d_T;

    // last sprine at single -> double 
    if(zero_start){
        cpn_d_0 = {0.0f, 0.0f};
        cvn_d_0 = {0.0f, 0.0f};
        can_d_0 = {0.0f, 0.0f};
        this->T_ds = this->T_ds/2;
    }else{
        cpn_d_0 =
            model.calc_LIP_p(
                this->T_sup_x - this->T_ds/2,
                this->T_sup   - this->T_ds/2,
                {-this->pn[0], -this->pn[1]},
                this->cvn_start
            );

        array<float,2> cpn_last =
            model.calc_LIP_p(
                this->T_sup_x,
                this->T_sup,
                {-this->pn[0], -this->pn[1]},
                this->cvn_start
            );
        cpn_d_0 = {cpn_d_0[0]-cpn_last[0], cpn_d_0[1]-cpn_last[1]};

        cvn_d_0 =
            model.calc_LIP_v(
                this->T_sup_x - this->T_ds/2,
                this->T_sup   - this->T_ds/2,
                {-this->pn[0], -this->pn[1]},
                this->cvn_start
            );

        can_d_0 = {cpn_d_0[0]/(Tc*Tc), cpn_d_0[1]/(Tc*Tc)};
    }

    // calculate variables for next sprine
    // update cn_start
    this->cvn_start = this->cvn_last;
    
    // update foot positions
    this->pn = this->pn_p1;
    this->p_n2m1 = {-this->p_n2p1[0], -this->p_n2p1[1]};
    this->p_n2m1 = model.rotate_vec(this->p_n2m1, -this->body_angle);

    // rotate and convert cn_d_0
    cpn_d_0 = model.rotate_vec(cpn_d_0, -this->body_angle);
    cpn_d_0 = {cpn_d_0[0]-this->pn[0], cpn_d_0[1]-this->pn[1]};
    cvn_d_0 = model.rotate_vec(cvn_d_0, -this->body_angle);
    can_d_0 = model.rotate_vec(can_d_0, -this->body_angle);

    // calculate cn_d_T
    if(zero_end){
        cpn_d_T = {0.0f, model.get_foot_dist_y_base()*this->pivot_sign(this->pivot)};
        cvn_d_T = {0.0f, 0.0f};
        can_d_T = {0.0f, 0.0f};
        this->T_ds = this->T_ds/2;
    }else{
        cpn_d_T =
            model.calc_LIP_p(
                this->T_ds/2,
                this->T_ds/2,
                {-this->pn[0], -this->pn[1]},
                this->cvn_start
            );

        cvn_d_T =
            model.calc_LIP_v(
                this->T_ds/2,
                this->T_ds/2,
                {-this->pn[0], -this->pn[1]},
                this->cvn_start
            );

        can_d_T = {cpn_d_T[0]/(Tc*Tc), cpn_d_T[1]/(Tc*Tc)};
    }

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
    array<float, 2> cvn_last_local = model.calc_LIP_v(
        this->T_sup_x,
        this->T_sup,
        {-this->pn[0], -this->pn[1]}, 
        this->cvn_start
    );
    cvn_last_local = model.rotate_vec(cvn_last_local, -this->body_angle);

    this->pn_p1 = model.foot_pos_pd(
        cvn_last_local, this->cvn_start, 
        vd, 
        this->pn, 
        this->T_sup_x, this->T_sup
    );
}

void GaitController::init_side(array<float, 3> vd){
    // initialize T_sup_x
    float T_sup_side_small = 0.17f;
    float T_sup_side_large = 0.17f;
    if (pivot_sign(this->pivot) != int(vd[1]/abs(vd[1]))){
        this->T_sup = T_sup_side_small;
        this->T_sup_next = T_sup_side_large;
    }else{
        this->T_sup = T_sup_side_large;
        this->T_sup_next = T_sup_side_small;
    }
    this->T_sup_x = this->T_sup;
    // calculate variables for next sprine
    // update cn_start
    this->cvn_start = this->cvn_last;
    
    // update foot positions
    this->pn = this->pn_p1;
    this->p_n2m1 = {-this->p_n2p1[0], -this->p_n2p1[1]};
    this->p_n2m1 = model.rotate_vec(this->p_n2m1, -this->body_angle);

    // Serial.println("---------------init_side--------------");
    // Serial.print("T_sup: "); Serial.println(this->T_sup, 4);
    // Serial.print("pn: "); Serial.print(this->pn[0], 4); Serial.print(", "); Serial.println(this->pn[1], 4);
    // Serial.print("p_n2m1: "); Serial.print(this->p_n2m1[0], 4); Serial.print(", "); Serial.println(this->p_n2m1[1], 4);
    // Serial.print("cvn_start: "); Serial.print(this->cvn_start[0], 4); Serial.print(", "); Serial.println(this->cvn_start[1], 4);
    this->T_ds = 0.0f;
}

void GaitController::update_state_variables_side(array<float, 3> vd){
    // Serial.println("---------------update_state_variables_side--------------");
    // normalize control input
    vd = model.normalize_vel(vd);

    // decide angle
    array<float, 2> angle_limits = model.calc_rot_angle_limit(this->cvn_start);
    float rotation = vd[2];
    if (rotation > 0) {
        rotation = angle_limits[0] * abs(rotation);
    } else {
        rotation = angle_limits[1] * abs(rotation);
    }
    this->body_angle = rotation;

    array<float, 2> cvn_last_local = model.calc_LIP_v(
        this->T_sup,
        this->T_sup,
        {-this->pn[0], -this->pn[1]}, 
        this->cvn_start
    );
    cvn_last_local = model.rotate_vec(cvn_last_local, -this->body_angle);

    float Tc = model.get_Tc();
    // decide pn_p1
    // y_pos_calculation
    // if (this->pivot == Pivot::RIGHT){
    //     Serial.println("pivot: RIGHT");
    // }else{
    //     Serial.println("pivot: LEFT");
    // }

    if (pivot_sign(this->pivot) != int(vd[1]/abs(vd[1]))){
        float basic_foot_y = model.get_foot_dist_y_base();
        float vd_base = model.calc_basic_unpassing_com_vel(basic_foot_y, T_sup);
        vd[1] = vd[1] * 1.2 * abs(vd_base);

        // Serial.println("pivot side");
        this->pn_p1[1] = -1 * Tc / sinh(this->T_sup_next/Tc) * (vd[1] - cvn_last_local[1]*cosh(this->T_sup_next/Tc));
    }else{
        // Serial.println("non-pivot side");
        // foot pos that COM stops at t=T_sup
        // Serial.print("cvn_last_local[1]: "); Serial.println(cvn_last_local[1], 4);
        this->pn_p1[1] = model.calc_basic_unpassing_foot_pos(cvn_last_local[1], this->T_sup_next * 1.7f);
        // Serial.print("pn_p1[1]: "); Serial.println(pn_p1[1], 4);
    }

    // Serial.print("vd[1]: "); Serial.println(vd[1], 4);
    // Serial.print("cvn_last_local: "); Serial.print(cvn_last_local[0], 4); Serial.print(", "); Serial.println(cvn_last_local[1], 4);
    
    // x_pos calculation
    array<float, 2> pn_p1_local = model.foot_pos_pd(
        cvn_last_local, this->cvn_start, 
        {0, vd[1], vd[2]},
        this->pn, 
        this->T_sup_next, this->T_sup_next
    );
    this-> pn_p1[0] = pn_p1_local[0];

    // Serial.print("T_sup: "); Serial.println(this->T_sup, 4);
    // Serial.print("pn_p1: "); Serial.print(this->pn_p1[0], 4); Serial.print(", "); Serial.println(this->pn_p1[1], 4);
    // Serial.println("-----------------------------");
}

/* #########################################################################
CALCULATION of SWING LEG
calculate start / half / last position of swing leg
##########################################################################*/
void GaitController::init_single(){
    // reinitialize T_ds
    // this->T_ds = this->T_sup * this->ds_ratio;
    // calculate model approximation coefficients
    model.calc_approx_coeff_y(-this->pn[1], this->cvn_start[1], this->T_sup);

    this->pivot_leg_angle  = 0.0;
    this->swing_leg_angle  = this->body_angle;
    this->single_start_com = this->model.calc_LIP_p(
        this->T_ds/2,
        this->T_ds/2,
        {-this->pn[0], -this->pn[1]},
        this->cvn_start
    );
    this->swing_com_0 = {
        -this->p_n2m1[0] + this->single_start_com[0],
        -this->p_n2m1[1] + this->single_start_com[1]
    };
    this->swing_com_0 = model.rotate_vec(this->swing_com_0, this->body_angle);

    array<float, 2> pivot_com_half = model.calc_LIP_p(
        this->T_sup_x*0.5f,
        this->T_sup  *0.5f,
        {-this->pn[0], -this->pn[1]}, 
        this->cvn_start
    );
    float pivot_com_diff_y = pivot_com_half[1] - this->single_start_com[1];
    this->swing_com_half = {0.0f, this->swing_com_0[1] + pivot_com_diff_y};
}

void GaitController::calc_swing_last(){
    // update cvn_last
    this->cvn_last = model.calc_LIP_v(
        this->T_sup_x,
        this->T_sup,
        {-this->pn[0], -this->pn[1]}, 
        this->cvn_start
    );
    this->cvn_last = model.rotate_vec(this->cvn_last, -this->body_angle);
    // update p_n2p1
    array<float, 2> cpn_last = model.calc_LIP_p(
        this->T_sup_x,
        this->T_sup,
        {-this->pn[0], -this->pn[1]}, 
        this->cvn_start
    );
    array<float, 2> pn_p1_rot = model.rotate_vec(this->pn_p1, this->body_angle);
    this->p_n2p1 = {
        cpn_last[0]+pn_p1_rot[0],
        cpn_last[1]+pn_p1_rot[1]
    };

    // calculate com at the end of single support
    array<float, 2> single_last_com = this->model.calc_LIP_p(
        this->T_sup_x - this->T_ds/2,
        this->T_sup   - this->T_ds/2,
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
array<array<float, 5>, 3> GaitController::calc_com_traj_single(bool calculated, float tx, float ty){
    float T_ss = this->T_sup - this->T_ds;

    // calculate com position for each t
    array<float, 2> com = model.calc_LIP_p(
        tx + this->T_ds/2,
        ty + this->T_ds/2,
        {-this->pn[0], -this->pn[1]}, 
        this->cvn_start
    );
    
    // calculate com z based on ty for stable walking
    array<float, 2> com_z = model.calc_com_z(ty, this->T_sup, this->ds_ratio);
    array<float, 2> swing_com;

    // calculate swing foot trajectory based on ty to synchronize with side way phase
    if (!calculated){
        swing_com[0] = this->swing_com_0[0] * (T_ss*0.5f - ty) + this->swing_com_half[0] * ty;
        swing_com[1] = this->swing_com_0[1] * (T_ss*0.5f - ty) + this->swing_com_half[1] * ty;
        swing_com = {swing_com[0]/(T_ss*0.5f), swing_com[1]/(T_ss*0.5f)};
        this->swing_leg_angle = this->body_angle * (T_ss*0.5f - ty) / (T_ss*0.5f);
    }else{
        this->calc_swing_last();
        swing_com[0] = this->swing_com_half[0] * (T_ss - ty) + this->swing_com_last[0] * (ty - T_ss*0.5f);
        swing_com[1] = this->swing_com_half[1] * (T_ss - ty) + this->swing_com_last[1] * (ty - T_ss*0.5f);
        swing_com = {swing_com[0]/(T_ss - T_ss*0.5f), swing_com[1]/(T_ss - T_ss*0.5f)};
        this->pivot_leg_angle = this->body_angle * (ty - T_ss*0.5f) / (T_ss - T_ss*0.5f);
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

array<array<float, 5>, 3> GaitController::get_default_com_pos(){
    float y = model.get_foot_dist_y_base() * this->pivot_sign(this->pivot);
    array<float, 5> com_pivot = {0, y, model.get_z0(), 0, 0};
    array<float, 5> com_else = {0, -y, model.get_z0(), 0, 0};

    if(pivot == Pivot::RIGHT){
        return {com_pivot, com_else, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
    }else{
        return {com_else, com_pivot, {0.0f, 0.0f, 0.0f, 0.0f, 0.0f}};
    }
}
