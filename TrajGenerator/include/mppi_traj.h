#ifndef PP_MPPI_TRAJ_H
#define PP_MPPI_TRAJ_H

#include "TrajectoryPlanner.h"


class MPPI_Traj : public TrajectoryPlanner{

MPPI_Traj(){};
MPPI_Traj(uint32_t traj_num, uint32_t pred_steps, float pred_time, float model_update_dt);
~MPPI_Traj();
virtual void TrajGenerate(double vCur, double omegaCur, double xCur, double yCur, double yawCur);
virtual void TrajGenerate();



void UpdateInputSequence(const std::vector<CTwist2D>& input_seq);

private:
std::vector<std::vector<CTwist2D> >input_seq_list_;

std::queue<CTwist2D> init_input_seq_;  //size: same as the number of steps

std::vector<CTwist2D> opt_input_seq_;  //size: same as the number of steps



        
};
















#endif // PP_MPPI_TRAJ_H