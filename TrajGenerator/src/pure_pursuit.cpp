#include "pure_pursuit.h"
#include <algorithm>
#include "pid.h"


PurePursuit::PurePursuit(PlannerMap* plannermap){
    SetMapPtr(plannermap);
    look_ahead_time_ = 0.4;
    look_ahead_distance_ = 0.2;
    min_look_ahead_distance_ = 0.1;
    max_look_ahead_distance_ = 0.8;
    curvature_ = 0.0;
    v_cmd_ = 0.0;
    omega_cmd_ = 0.0;
    v_cur_ = 0.0;
}


double PurePursuit::getLookAheadDistance(){
    double look_ahead_distance = abs(cur_state_.GetVel()) * look_ahead_time_;
    return look_ahead_distance;
}


// void PurePursuit::CalculateCurvature(){
//     double look_ahead_distance = getLookAheadDistance();
//     getLookAheadPoint();

//     double look_ahead_x = target_lane_x_;
//     double dx = look_ahead_x - cur_pose_.x;
//     double dy = look_ahead_y - cur_pose_.y;
//     double yaw = cur_pose_.th;
//     double delta_yaw = look_ahead_yaw - yaw;
//     double curvature = 2 * dy / (dx * dx + dy * dy);
//     curvature_ = curvature;
//     v_cmd_ = v_cur_;
//     omega_cmd_ = v_cur_ * curvature;
// }

void PurePursuit::ComputeTwistCmd(){
    PID pid(1.4, 0.0, 0.0, 0.1, -0.1);
    double target_yaw = GetTargetYaw();
    double omega_theta_e = pid.Calculate(cur_pose_.th, target_yaw, 0.1);
    cout<<"omega_theta_e: "<<omega_theta_e<<endl;
    Point2dT look_ahead_point;
    CalculateCurvature(look_ahead_point);
    v_cmd_ = cur_state_.GetVel();
    cout<<"v_cmd_: "<<v_cmd_<<endl;
    double omega_ye = v_cmd_ * curvature_;
    omega_cmd_ = omega_theta_e + omega_ye;
}


void PurePursuit::CalculateCurvature(Point2dT look_ahead_point){
    double look_ahead_distance = getLookAheadDistance();
    double dlt_x = cur_pose_.x - target_lane_x_;
    cout<<"dlt_x: "<<dlt_x<<endl;
    double dlt_y = sqrt(look_ahead_distance * look_ahead_distance - dlt_x * dlt_x);

    // const double carrot_dist2 = dlt_x * dlt_x + dlt_y * dlt_y;
    if(fabs(dlt_x) > 0.005){
        curvature_ = 2 * dlt_x / look_ahead_distance;
    }
    else{
        curvature_ = 0.0;
    }
}

bool PurePursuit::CheckGoal(){
    double dlt_x = cur_state_.GetX() - target_lane_x_;
    double theta_e = cur_state_.GetYaw() - target_yaw_;
    if(fabs(dlt_x) < 0.005 && fabs(theta_e) < 0.01){
        return true;
    }
    else{
        return false;
    }
    
}


