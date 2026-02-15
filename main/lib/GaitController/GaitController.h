#include <Arduino.h>
#include <cmath>
#include "SLIP.h"

using std::array;

enum class Pivot : uint8_t{
    RIGHT,
    LEFT
};

class GaitController{
public:
    GaitController();
    // initialization
    void set_cpn_start(array<float, 2> cpn){ this->cpn_start = cpn; }
    void set_cvn_start(array<float, 2> cvn){ this->cvn_start = cvn; }
    void set_cvn_m1_start(array<float, 2> cvn_m1){ this->cvn_m1_start = cvn_m1; }
    void set_cvn_last(array<float, 2> cvn){ this->cvn_last = cvn; }
    void set_pn(array<float, 2> pn){ this->pn = pn; }
    void set_pn_p1(array<float, 2> pn_p1){ this->pn_p1 = pn_p1; }
    void set_p_n2p1(array<float, 2> p_n2p1){ this->p_n2p1 = p_n2p1; }
    void set_p_n2m1(array<float, 2> p_n2m1){ this->p_n2m1 = p_n2m1; }
    void set_T_sup(float T_sup){ this->T_sup = T_sup; }
    void set_T_sup_next(float T_sup_next){ this->T_sup_next = T_sup_next; }
    void set_T_sup_last(float T_sup_last){ this->T_sup_last = T_sup_last; }
    void set_T_ds(float T_ds){ this->T_ds = T_ds; }
    void set_ds_ratio(float ds_ratio){ this->ds_ratio = ds_ratio; }
    void set_body_angle(float body_angle){ this->body_angle = body_angle; }

    void init_param_walk();
    void init_param_fight();

    // pivot
    void inverse_pivot();
    int pivot_sign(Pivot p);

    // update state variables
    void init_state_variables();
    void update_state_variables(array<float, 3> vd, array<float, 2> foot_pos_fb);

    // update gait trajectory param
    void init_start();
    void init_end();
    void init_single_0();
    void init_single_half();
    void init_double();

    // calculate trajectory for each steps
    array<array<float, 5>, 2> calc_com_traj_single(float t);
    array<array<float, 5>, 2> calc_com_traj_double(float t);

    // getters
    float get_T_sup(){ return this->T_sup; }
    float get_T_ds(){ return this->T_ds; }
    float get_ds_ratio(){ return this->ds_ratio; }

private:
    // model
    SLIP model;

    // pivot
    Pivot pivot;
    
    // state variables
    // com state at start and last (no consideration about double support phase)
    array<float, 2> cpn_start;
    array<float, 2> cvn_start;
    array<float, 2> cvn_m1_start;
    array<float, 2> cvn_last;

    // foot pos
    array<float, 2> pn;
    array<float, 2> pn_p1;
    array<float, 2> p_n2p1;
    array<float, 2> p_n2m1;

    // phase time
    float T_sup;
    float T_sup_next;
    float T_sup_last;

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
