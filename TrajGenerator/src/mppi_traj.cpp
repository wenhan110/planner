#include "mppi_traj.h"

MPPI_Traj::MPPI_Traj(uint32_t traj_num, uint32_t pred_steps, float pred_time, float model_update_dt){
    traj_num_ = traj_num;
    pred_steps_ = pred_steps;
    pred_time_ = pred_time;
    model_update_dt_ = model_update_dt;

}


MPPI_Traj::~MPPI_Traj(){

}


 