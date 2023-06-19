#include <iostream>
#include <math.h>
#include "MotionModel.h"


using namespace std;

MotionModel::MotionModel(double x, double y, double yaw, double v, double omega, double model_update_dt)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    v_ = v;
    omega_ = omega;
    yaw_last_ = yaw;
    model_update_dt_ = model_update_dt;
}



void MotionModel::UpdateWithTL(const double& v, const double& omega)
{
    // double yaw = cur_state_.yaw_;
    // cur_state_.x_ += v * std::cos(yaw) * model_update_dt_;
    // cur_state_.y_ += v * std::sin(yaw) * model_update_dt_;
    // cur_state_.yaw_ += omega * model_update_dt_;
    // cur_state_.v_ = v;
    // cur_state_.omega_ = omega;
    x_ += v * std::cos(yaw_) * model_update_dt_;
    y_ += v * std::sin(yaw_) * model_update_dt_;
    yaw_ += omega * model_update_dt_;
    v_ = v;
    omega_ = omega;
}


void MotionModel::UpdateWithSL(const double& v, const double& omega)    
{
    // double yaw = cur_state_.yaw_ + omega * model_update_dt_ / 2;
    // cur_state_.x_ += v * std::cos(yaw) * model_update_dt_;
    // cur_state_.y_ += v * std::sin(yaw) * model_update_dt_;
    // cur_state_.yaw_ += omega * model_update_dt_;
    // cur_state_.v_ = v;
    // cur_state_.omega_ = omega;
    double yaw = yaw_ + omega * model_update_dt_ / 2;
    x_ += v * std::cos(yaw) * model_update_dt_;
    y_ += v * std::sin(yaw) * model_update_dt_;
    yaw_ += omega * model_update_dt_;
    v_ = v;
    omega_ = omega;
}


void MotionModel::UpdateWithCircleModel(const double& v, const double& omega)
{
    if(omega == 0){
        UpdateWithTL(v, omega);
    }
    else{
        yaw_ = yaw_ + omega * model_update_dt_;
        double R = v / omega;
        // double yaw = yaw_ + omega * model_update_dt_;
        x_ += R * (std::sin(yaw_) - std::sin(yaw_last_));
        y_ += R * (std::cos(yaw_last_) - std::cos(yaw_));
        v_ = v;
        omega_ = omega;
        // yaw_last_ = yaw_;
    }
    yaw_last_ = yaw_;
}

void MotionModel::UpdateWithAckerMann(const double& v, const double& omega)  //todo: minmum radius for pattern
{}


