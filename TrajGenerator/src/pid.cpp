#include "pid.h"
#include <algorithm>

PID::PID(double kp, double ki, double kd, double error_integration_max, double error_integrartion_min):
            Kp_(kp), Ki_(ki), Kd_(kd),
            error_integration_max(error_integration_max), error_integration_min(error_integrartion_min),
            error_cur_(0.0), error_last_(0.0), error_integration_(0.0), error_derivative_(0.0){}


void PID::SetKp_variant(double Kp_max, double Kp_min, double error_min, double error_max){
    double abs_error = fabs(error_cur_);
    if(abs_error >= error_max){
        Kp_ = Kp_min;
    }
    else if(abs_error <= error_min){
        Kp_ = Kp_max;
    }
    else{
        Kp_ = Kp_min *(abs_error - error_min) / (error_max - error_min) + Kp_max * (error_max - abs_error) / (error_max - error_min);
    }
}


double PID::Calculate(double output, double setpoint, double dt){
    error_cur_ = setpoint - output;
    error_derivative_ = (error_cur_ - error_last_) / dt;
    error_last_ = error_cur_;
    error_integration_ += error_cur_ * dt;
    if(error_cur_ * error_last_ < 0){
        error_integration_ = 0.0;
    }
    error_integration_ = std::min(std::max(error_integration_, error_integration_min), error_integration_max);
    double cmd =  Kp_ * error_cur_ + Ki_ * error_integration_ + Kd_ * error_derivative_;
    return cmd;
}

