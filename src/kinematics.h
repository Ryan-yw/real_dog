#ifndef KINEMATICS_H_
#define KINEMATICS_H_
// 连杆长度 //
const double L1 = 0.1315;  //单位m
const double L2 = 0.306;
const double L3 = 0.340;
const double PI = 3.14159265358979323846;

//身体长宽高设置
const double kBodyLong = 0.652; //m  x方向
const double kBodyWidth = 0.1250;   //m  z方向
const double kBodyHigh = 0.500;    //m y方向

auto inverseSame(double* leg_in_ground, double* body_in_ground, double* input)->int;
auto inverseSymmetry(double* leg_in_ground, double* body_in_ground, double* input)->int;
double* s_pm_dot_pm(const double* pm1, const double* pm2, double* pm_out);
double* s_inv_pm_dot_pm(const double* inv_pm, const double* pm, double* pm_out);
#endif
 
