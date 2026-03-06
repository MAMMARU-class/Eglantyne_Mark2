#include <Arduino.h>
#include <cmath>
#include "SLIP.h"

#define HEIGHT_WALK 0.142f
#define HEIGHT_FIGHT 0.135f

#define HEIGHT_CROUCH 0.09f
#define PHI_CROUCH 25.0f * PI / 180.0f

#define HEIGHT_GUARD 0.12f

#define HEIGHT_JUMP 0.1f
#define PHI_JUMP 20.0f * PI / 180.0f
#define HEIGHT_UPDATE_RATE 0.001f

using std::array;

enum class Pivot : uint8_t{
    RIGHT,
    LEFT
};

class GaitController{
public:
    GaitController();
    /* #########################
    CALCULATION PARAMETERS
    ##########################*/
    // initialization
    void set_vd_max_abs(array<float, 3> vd_max_abs){ this->vd_max_abs = vd_max_abs; }
    void set_cpn_start(array<float, 2> cpn){ this->cpn_start = cpn; }
    void set_cvn_start(array<float, 2> cvn){ this->cvn_start = cvn; }
    void set_cvn_last(array<float, 2> cvn){ this->cvn_last = cvn; }
    void set_pn(array<float, 2> pn){ this->pn = pn; }
    void set_pn_p1(array<float, 2> pn_p1){ this->pn_p1 = pn_p1; }
    void set_p_n2p1(array<float, 2> p_n2p1){ this->p_n2p1 = p_n2p1; }
    void set_p_n2m1(array<float, 2> p_n2m1){ this->p_n2m1 = p_n2m1; }
    void set_T_sup(float T_sup){ this->T_sup = T_sup; }
    void set_T_ds(float T_ds){ this->T_ds = T_ds; }
    void set_ds_ratio(float ds_ratio){ this->ds_ratio = ds_ratio; }
    void set_body_angle(float body_angle){ this->body_angle = body_angle; }

    void init_param_walk(float z0);
    void init_param_side(float z0);
    void init_param_crouch(float z0);
    void init_param_fight(float z0);
    void init_pose();

    // pivot
    void inverse_pivot();
    int pivot_sign(Pivot p);

    /* #########################
    STATE VARIABLES
    ##########################*/
    // update state variables
    void init_state_variables(bool zero_start = false, bool zero_end = false);
    void update_state_variables(array<float, 3> vd);
    void init_side(array<float, 3> vd);
    void update_state_variables_side(array<float, 3> vd);
    void feedback_x0_vx0(array<float, 2> x0_vx0_fb){
        this->pn_p1[0] -= x0_vx0_fb[0];
        this->pn[0] -= x0_vx0_fb[0];
        this->cvn_start[0] -= x0_vx0_fb[1];
    }
    void update_T_sup_x(float increment){
        this->T_sup_x += increment;
    }

    /* #########################
    CALCULATION of SWING LEG
    ##########################*/
    void init_single();
    void calc_swing_last();

    /* #########################
    TRAJECTORY CALCULATION
    ##########################*/
    // calculate trajectory for each steps
    array<array<float, 5>, 3> calc_com_traj_single(bool calculated, float tx, float ty);
    array<array<float, 5>, 3> calc_com_traj_double(float t);
    array<array<float, 5>, 3> get_default_com_pos();

    // getters
    float get_T_sup(){ return this->T_sup; }
    float get_T_ds(){ return this->T_ds; }
    float get_ds_ratio(){ return this->ds_ratio; }
    bool pivot_right(){ return this->pivot == Pivot::RIGHT; }
    bool p_n2p1_equels_p_n2m1(){
        return (
            abs(this->p_n2p1[0] + this->p_n2m1[0]) < 1e-3f &&
            abs(this->p_n2p1[1] + this->p_n2m1[1]) < 1e-3f
        );
    };
    array<float, 3> get_vd_max_abs(){ return this->vd_max_abs; }
    // getters from model
    float get_Tc(){ return model.get_Tc(); }
    array<float, 3> get_approx_coeff_y(){ return model.get_approx_coeff_y(); }
    bool is_pivot_right(){ 
        if (pivot == Pivot::RIGHT){ return true; 
        }else{ return false; }
    }
    array<float, 2> get_x0_vx0(){
        return {-this->pn[0], this->cvn_start[0]};
    }
    array<float, 2> rotate_vec(const array<float, 2>& vec, float angle){
        return model.rotate_vec(vec, angle);
    }
    float get_foot_dist_y_base(){ return model.get_foot_dist_y_base(); }

private:
    // model
    SLIP model;

    // pivot
    Pivot pivot;
    // v input
    array<float, 3> vd_max_abs = {0.02f, 0.02f, 0.2f};
    
    // state variables
    // com state at start and last (no consideration about double support phase)
    array<float, 2> cpn_start;
    array<float, 2> cvn_start;
    array<float, 2> cvn_last;

    // foot pos
    array<float, 2> pn;
    array<float, 2> pn_p1;
    array<float, 2> p_n2p1;
    array<float, 2> p_n2m1;

    // phase time
    float T_sup;
    float T_sup_x;
    float T_sup_next;

    // double support calculation
    float T_ds;
    float ds_ratio;

    // angle
    float body_angle;

    // single support phase
    float pivot_leg_angle;
    float swing_leg_angle;

    array<float, 2> single_start_com;
    array<float, 2> swing_com_0;
    array<float, 2> swing_com_half;
    array<float, 2> swing_com_last;
};
