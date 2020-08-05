#ifndef KINEMATICS_H_
#define KINEMATICS_H_

const double L1 = 72.4;  //mm
const double L2 = 313.73;
const double L3 = 335.70;
const double PI = 3.14159265358979323846;
//const double kBodyLong = 829.19; //mm  x方向
const double kBodyLong = 829.19; //mm  x方向
const double kBodyWidth = 150.2;   //mm  z方向
const double kBodyHigh = 550;    //mm  y方向
//const double kBodyHigh = L2+L3-0.01;    //mm  y方向
auto inverse(double* leg_in_ground, double* body_in_ground, double* input)->int;
double* s_pm_dot_pm(const double* pm1, const double* pm2, double* pm_out);
double* s_inv_pm_dot_pm(const double* inv_pm, const double* pm, double* pm_out);
#endif