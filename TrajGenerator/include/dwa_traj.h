#ifndef PP_DWA_TRAJ_H
#define PP_DWA_TRAJ_H

#include <queue>
#include "PlannerMap.h"
#include "config.h"
#include "MotionModel.h"


// struct MotionModel{
//     double x_, y_, yaw_;
//     double v_, dYaw_;
// };

class DWATraj{
public:
DWATraj(){}
DWATraj(PlannerMap* plannermap, uint32_t traj_num, uint32_t pred_steps, double pred_time, double model_update_dt);
~DWATraj();

void SetMapPtr(PlannerMap* plannermap);

void TrajGenerate(double vCur, double omegaCur, double xCur, double yCur, double yawCur);
void TrajGenerate(MotionModel& cur_state);
void GetOptimalTraj();
void VisualizeTraj();  
void VisualizeOptimalTraj();
void ComputeTwistCommand();
void DrawOptimalTraj();
void VisualizeTraj(int index);



// PlannerMap *local_map_;
PlannerMap *visual_map_;

ObstacleLayer* obs_layer_;
InflationLayer* obs_inflation_layer_;

// Pose2dT cur_pose_;
std::vector<PlannerMap*> visual_map_list_;
Matrix GetInflationMap(){
    return obs_inflation_layer_->GetCostmap();
}

MotionModel cur_state_;

void GetTwistCmd(double& v, double& omega){
    v = v_cmd_;
    omega = omega_cmd_;
}
void SetCurState(const MotionModel& state){
    cur_state_ = state;
}

void GetCurState(MotionModel& state){
    state = cur_state_;
}
void UpdateOptimalCmd();

void SetGoalPoint(Point2dT goal_point){
    global_goal_point_.x = goal_point.x;
    global_goal_point_.y = goal_point.y;
}

Pose2dT cur_pose_;

private:
// MotionModel cur_state_;


// void UpdateOptimalCmd();

void GetOptimalControl(std::queue<CTwist2D> &queue);

void UpdateInputRange();
// void GetOptimalTraj();

void ComputeCost(std::vector<MotionModel>& traj, std::vector<double>& cost_list);
void GenerateInputSequence(std::vector<CTwist2D>& list);
void MotionModelUpdate(MotionModel &state, const double& v, const double& omega);
void TrajPredict(const double& v, const double& omega, std::vector<MotionModel>& traj);



// PlannerMap *local_map_;
// PlannerMap *visual_map_;



// Pose2dT cur_pose_;
double vCur_, omegaCur_;
double vPrev_, omegaPrev_;
double model_update_dt_;  //0.05s
double v_list_max_, v_list_min_;
double omega_list_max_, omega_list_min_;
uint32_t pred_steps_;  //100 steps
double pred_time_;  //5s
uint32_t traj_num_;  //13

std::queue<CTwist2D> calculated_input_seq_;
std::vector<MotionModel> optimal_traj_;
std::vector<std::vector<MotionModel> > pred_traj_;
// std::vector<double> v_list;
// std::vector<double> dYaw_list;
std::vector<CTwist2D> input_seq_list_;
std::vector<double> traj_cost_list_;
Point2dT global_goal_point_;

bool isValidTraj_;

double weight_follow_ = 1000000000;
double weight_obs_ = 1.0;
double weight_goal_ = 1.0;
double weight_smooth_ = 1;
double weight_twirling_ = 100;

double v_cmd_, omega_cmd_;
};







#endif // PP_DWA_TRAJ_H