#ifndef PP_PLANNER_H
#define PP_PLANNER_H

#include "PlannerMap.h"
#include "a_star.h"
#include "dwa_traj.h"
#include "pure_pursuit.h"

class Planner{
public:

Planner();
~Planner();

void UpdateGlobalPath();
//FOr real battle
void UpdateCurPose(const Pose2dT& cur_pose);

void CurStateCallback(const MotionModel& cur_state){
    cur_state_ = cur_state;
}

void UpdateGoalPose(const Pose2dT& goal_pose);

void UpdateStartPose(const Pose2dT& start_pose);

void SetCurrentState(const MotionModel& cur_state){
    cur_state_ = cur_state;
}

void CalcTwistCmd();

void SetStartGoal();

uint32_t GetStartRow(){
    return global_start_.row_;
}   

uint32_t GetStartCol(){
    return global_start_.col_;
}

uint32_t GetGoalRow(){
    return global_goal_.row_;
}

uint32_t GetGoalCol(){
    return global_goal_.col_;
}

void MotionModelUpdate(const double v_cmd, const double omega_cmd);

PlannerMap* GetPlannerMap(){
    return planner_map_;
}

AStar* GetAStar(){
    return a_star_;
}

Pose2dT GetCurPose(){
    return cur_pose_;
}

DWATraj* GetDWATraj(){
    return dwa_traj_;
}

PurePursuit* GetPurePursuit(){
    return pure_pursuit_;
}

MotionModel &GetCurState(){
    return cur_state_;
}

Point2dT GetLocalGoal(){
    return local_goal_;
}

void VisualLocalRefPath();

void VisualizeLocalTrajectory();

void VisualizeGlobalPath();

void PruneGlobalPath();

void TransformGlobalPath();

bool CheckGlobalGoal();

private:

PlannerMap* planner_map_;
AStar* a_star_;
DWATraj* dwa_traj_;
PurePursuit* pure_pursuit_;
Pose2dT cur_pose_;
Pose2dT goal_pose_;
Pose2dT start_pose_;
MotionModel cur_state_;
CTwist2D twist_cmd_;
std::vector<Cell> global_path_;
std::vector<Point2dT> global_path_points_;
std::vector<Point2dT> local_path_points_;
Point2dT local_goal_;
Cell global_start_;
Cell global_goal_;





};



#endif // PP_PLANNER_H