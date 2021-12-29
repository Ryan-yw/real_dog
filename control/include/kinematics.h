#ifndef KINEMATICS_H_
#define KINEMATICS_H_
// 连杆长度 //
const double L1 = 0.134;  //单位m
const double L2 = 0.306;
const double L3 = 0.310;
const double PI = 3.14159265358979323846;


//身体长宽高设置
const double kBodyLong = 0.60398; //m  x方向
const double kBodyWidth = 0.126;   //m  z方向
const double kBodyHigh = 0.480;    //m y方向


//初始时身体和脚在地面坐标系下的位置
static double foot_position_start_point[12] = {
                                         kBodyLong / 2 -0.1, -kBodyHigh, -(kBodyWidth / 2) - L1 ,  //leg1 ->012
                                        -kBodyLong / 2 -0.1, -kBodyHigh, -(kBodyWidth / 2) - L1,  //leg2 ->345
                                        -kBodyLong / 2 -0.1, -kBodyHigh,  (kBodyWidth / 2) + L1,   //leg3 ->678
                                         kBodyLong / 2 -0.1, -kBodyHigh,  (kBodyWidth / 2) + L1   //leg4 ->91011
};

//static double foot_position_start_point[12] = {
//                  00
//};
static double body_position_start_point[16] = { 1,0,0,0,
                                                0,1,0,0,
                                                0,0,1,0,
                                                0,0,0,1 };



//auto inverseSame(double* leg_in_ground, double* body_in_ground, double* input)->int;
//auto inverseSymmetry(double* leg_in_ground, double* body_in_ground, double* input)->int;

#endif
 
