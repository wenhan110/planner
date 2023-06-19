#ifndef PP_PURE_PURSUIT_H
#define PP_PURE_PURSUIT_H

#include "PlannerMap.h"
#include "MotionModel.h"

class PurePursuit{
public:
PurePursuit(){}
PurePursuit(PlannerMap *plannermap);
// PurePursuit(PlannerMap plannermap);
~PurePursuit(){}

void SetMapPtr(PlannerMap* plannermap){
    obs_layer_ = plannermap->GetObstacleLayer();
    obs_inflation_layer_ = plannermap->GetObsInflationLayer();
}

void SetCurState(const MotionModel& state){
    cur_state_ = state;
}

void SetCurPose(const Pose2dT& cur_pose){
    cur_pose_ = cur_pose;
}

void SetTargetLaneX(const double& target_lane_x){
    target_lane_x_ = target_lane_x;
}

double GetTargetLaneX(){
    return target_lane_x_;
}

void SetTargetYaw(const double& target_yaw){
    target_yaw_ = target_yaw;
}

double GetTargetYaw(){
    return target_yaw_;
}

void SetVcur(const double& v_cur){
    v_cur_ = v_cur;
}

void CalculateCurvature(Point2dT look_ahead_point);

void ComputeTwistCmd();

void getLookAheadPoint();

double getLookAheadDistance();

double getCurvature(){
    return curvature_;
}

double getVcmd(){
    return v_cmd_;
}

double getOmegacmd(){
    return omega_cmd_;
}

bool CheckGoal();

MotionModel cur_state_;
ObstacleLayer* obs_layer_;
InflationLayer* obs_inflation_layer_;
Pose2dT cur_pose_; 

private:

double look_ahead_time_, look_ahead_distance_;
double min_look_ahead_distance_, max_look_ahead_distance_;
double curvature_;
double v_cmd_, omega_cmd_;
double v_cur_;
double target_lane_x_;
double target_yaw_;

// ObstacleLayer* obs_layer_;
// InflationLayer* obs_inflation_layer_;


};






#endif // PP_PURE_PURSUIT_H