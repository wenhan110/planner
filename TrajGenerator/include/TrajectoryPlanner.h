#ifndef PP_TRAJECTORYPLANNER_H
#define PP_TRAJECTORYPLANNER_H
#include "PlannerMap.h"
#include "MotionModel.h"
#include <queue>
class TrajectoryPlanner{

public: 

TrajectoryPlanner(){}
TrajectoryPlanner(uint32_t traj_num, uint32_t pred_steps, float pred_time, float model_update_dt){}
virtual ~TrajectoryPlanner(){}

virtual void TrajGenerate(double vCur, double omegaCur, double xCur, double yCur, double yawCur){}
virtual void TrajGenerate(){}
virtual void GetOptimalTraj(){}

void GetTwistCmd(double& v, double& omega){
    v = vel_cmd_;
    omega = omega_cmd_;
}
void SetCurState(const MotionModel& state){
    cur_state_ = state;
}
void GetCurState(MotionModel& state){
    state = cur_state_;
}

virtual void UpdateOptimalCmd(){}

virtual void GetOptimalControl(std::queue<CTwist2D> &queue){}








protected:
std::vector<MotionModel> optimal_traj_;
std::vector<std::vector<MotionModel> > pred_traj_;
PlannerMap *local_map_;
PlannerMap *visual_map_;
MotionModel cur_state_;

uint32_t traj_num_;
uint32_t pred_steps_;
float pred_time_;
float model_update_dt_;
double vel_cmd_, omega_cmd_;





};










#endif // PP_TRAJECTORYPLANNER_H