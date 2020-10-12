#ifndef KINEMATICS_H_
#define KINEMATICS_H_

const double L1 = 131.5;  //mm
const double L2 = 306;
const double L3 = 340;
const double PI = 3.14159265358979323846;
const double kBodyLong = 652; //mm  x方向
const double kBodyWidth = 125.0;   //mm  z方向
const double kBodyHigh = 500;    //mm  y方向
auto inverseSame(double* leg_in_ground, double* body_in_ground, double* input)->int;
auto inverseSymmetry(double* leg_in_ground, double* body_in_ground, double* input)->int;
double* s_pm_dot_pm(const double* pm1, const double* pm2, double* pm_out);
double* s_inv_pm_dot_pm(const double* inv_pm, const double* pm, double* pm_out);
#endif
