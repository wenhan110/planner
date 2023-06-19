#ifndef PP_MOTIONMODEL_H
#define PP_MOTIONMODEL_H



// struct MotionModel{
//     double x_, y_, yaw_;
//     double v_, dYaw_;
// };



class MotionModel{
public:
MotionModel(){};
MotionModel(double x, double y, double yaw, double v, double omega, double model_update_dt);
// ~MotionModel();



void UpdateWithTL(const double& v, const double& omega);

void UpdateWithSL(const double& v, const double& omega);

void UpdateWithCircleModel(const double& v, const double& omega);

void UpdateWithAckerMann(const double& v, const double& omega);

double GetX(){return x_;}
double GetY(){return y_;}
double GetYaw(){return yaw_;}
double GetVel(){return v_;}
double GetOmega(){return omega_;}
double GetYawLast(){return yaw_last_;}


private:
double model_update_dt_;  //0.05s
double x_, y_, yaw_;
double v_, omega_;
double yaw_last_;
};







#endif // PP_MOTIONMODEL_H