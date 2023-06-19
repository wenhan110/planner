#ifndef PP_PID_H
#define PP_PID_H

#include <math.h>

class PID{
public:
PID(double Kp_ = 0.0, double Ki_ = 0.0, double Kd_ = 0.0, double error_integration_max = 100, double error_integration_min = 100);

void SetKp(double Kp){
    Kp_ = Kp_;
}

void SetKi(double Ki){
    Ki_ = Ki_;
}

void SetKd(double Kd){
    Kd_ = Kd_;
}

void SetKp_variant(double Kp_max, double Kp_min, double error_min, double error_max);

double reset_error_integration();

double getKp()const{
    return Kp_;
}

double getKi()const{
    return Ki_;
}

double getKd()const{
    return Kd_;
}

double getError()const{
    return error_cur_;
}

double getErrorIntegration()const{
    return error_integration_;
}

double getErrorDerivative()const{
    return error_derivative_;
}

double Calculate(double output, double setpoint, double dt);

double Calculate_Kp_variant(double output, double setpoint, double Kp_max, double Kp_min, double error_min, double error_max, double dt, int flag);





private:
double Kp_, Ki_, Kd_;

double error_cur_;
double error_last_;
double error_integration_;
double error_derivative_;
double error_integration_max;
double error_integration_min;





};



#endif // PP_PID_H