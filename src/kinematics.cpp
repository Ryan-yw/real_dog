#include"kinematics.h"
#include<cmath>
#include<iostream>
#include<fstream>
#include<aris.hpp>

extern double input_angle[12];


//身体在腿坐标系下的变换矩阵
double PL1[16] =
{
    -1, 0,  0,  kBodyLong / 2 ,
     0, 1,  0,  0           ,
     0, 0, -1, -kBodyWidth / 2,
     0, 0,  0,  1
};

double PL2[16] =
{
    -1, 0,  0, -kBodyLong / 2 ,
     0, 1,  0,  0           ,
     0, 0, -1, -kBodyWidth / 2,
     0, 0,  0,  1
};

double PL3[16] =
{
    1, 0, 0,  kBodyLong / 2 ,
    0, 1, 0,  0           ,
    0, 0, 1,  -kBodyWidth / 2,
    0, 0, 0,  1
};

double PL4[16] =
{
    1, 0, 0, -kBodyLong / 2 ,
    0, 1, 0,  0           ,
    0, 0, 1, -kBodyWidth / 2,
    0, 0, 0,  1
};


//运动学反解
void leg_12(double* ee_xyz_wrt_leg, double* mot_pos_3)
{
    //计算xita3
    double x = ee_xyz_wrt_leg[0];
    double y = ee_xyz_wrt_leg[1];
    double z = ee_xyz_wrt_leg[2];

    double A = 0;
    double B = 0;

    //计算xita1
    A = std::acos(std::abs(z) / std::hypot(y, z)) * 180 / PI;
    B = std::acos(L1 / std::hypot(y, z)) * 180 / PI;

    if (y <= 0 && z >= 0)
    {
        mot_pos_3[0] = A - B;
    }
    else if (y <= 0 && z <= 0)
    {
        mot_pos_3[0] = 180 - A - B;
    }
    else if (y > 0 && z > 0)
    {
        mot_pos_3[0] = A + B;
    }
    else if (y > 0 && z < 0)
    {
        mot_pos_3[0] = -180 + A - B;
    }

    //计算xita3

    mot_pos_3[2] = -180 + std::acos((L2 * L2 + L3 * L3 - (y * y + z * z - L1 * L1 + x * x)) / (2 * L2 * L3)) * 180 / PI;

    //计算xita2

    //判断大小腿坐标系

    A = PI / 2;
    B = std::sqrt(y * y + z * z - L1 * L1);
    y = -B;

    //计算xita2

    A = std::acos(std::abs(x) / std::hypot(x, y)) * 180 / PI;
    B = std::acos((L2 * L2 + x * x + y * y - L3 * L3) / (2 * L2 * std::hypot(x, y))) * 180 / PI;

    if (x <= 0 && y < 0)
    {
        mot_pos_3[1] = A + B;
    }

    if (x > 0 && y <= 0)
    {
        mot_pos_3[1] = 180 - A + B;
    }

    if (x <= 0 && y > 0)
    {
        mot_pos_3[1] = -A + B;
    }

    if (x > 0 && y >= 0)
    {
        mot_pos_3[1] = -180 + A + B;
    }

    mot_pos_3[1] -= 90;
    mot_pos_3[0] *= PI / 180.0;
    mot_pos_3[1] *= PI / 180.0;
    mot_pos_3[2] *= PI / 180.0;

}
void leg_34(double* ee_xyz_wrt_leg, double* mot_pos_3)
{
    //计算xita3

    double x = ee_xyz_wrt_leg[0];
    double y = ee_xyz_wrt_leg[1];
    double z = ee_xyz_wrt_leg[2];
    double A = 0;
    double B = 0;


    //计算xita1
    A = std::acos(std::abs(z) / std::hypot(y, z)) * 180 / PI;
    B = std::acos(L1 / std::hypot(y, z)) * 180 / PI;

    if (y <= 0 && z >= 0)
    {
        mot_pos_3[0] = A - B;
    }
    else if (y <= 0 && z <= 0)
    {
        mot_pos_3[0] = 180 - A - B;
    }
    else if (y > 0 && z > 0)
    {
        mot_pos_3[0] = A + B;
    }
    else if (y > 0 && z < 0)
    {
        mot_pos_3[0] = -180 + A - B;
    }

    //计算xita3

    mot_pos_3[2] = 180 - std::acos((L2 * L2 + L3 * L3 - (y * y + z * z - L1 * L1 + x * x)) / (2 * L2 * L3)) * 180 / PI;

    //计算xita2

    //判断大小腿坐标系

    A = PI / 2;
    B = std::sqrt(y * y + z * z - L1 * L1);
    y = -B;

    //计算xita2

    A = std::acos(std::abs(x) / std::hypot(x, y)) * 180 / PI;
    B = std::acos((L2 * L2 + x * x + y * y - L3 * L3) / (2 * L2 * std::hypot(x, y))) * 180 / PI;

    if (x <= 0 && y < 0)
    {
        mot_pos_3[1] = A - B;
    }
    else if (x > 0 && y <= 0)
    {
        mot_pos_3[1] = 180 - A - B;
    }

    mot_pos_3[1] -= 90;
    mot_pos_3[0] *= PI / 180.0;
    mot_pos_3[1] *= PI / 180.0;
    mot_pos_3[2] *= PI / 180.0;
}

auto inverseSame(double* leg_in_ground, double* body_in_ground, double* input)->int
{
    double real_pm1[16] = { 0 }, real_pm2[16] = { 0 }, real_pm3[16] = { 0 }, real_pm4[16] = { 0 };
    aris::dynamic::s_pm_dot_inv_pm(PL1, body_in_ground, real_pm1);
    aris::dynamic::s_pm_dot_inv_pm(PL2, body_in_ground, real_pm2);
    aris::dynamic::s_pm_dot_inv_pm(PL3, body_in_ground, real_pm3);
    aris::dynamic::s_pm_dot_inv_pm(PL4, body_in_ground, real_pm4);

    double xyz_in_leg[12] = { 0 }; //腿末端在腿坐标系下的表达
    aris::dynamic::s_pp2pp(real_pm1, leg_in_ground + 0 * 3, xyz_in_leg + 0 * 3);
    aris::dynamic::s_pp2pp(real_pm2, leg_in_ground + 1 * 3, xyz_in_leg + 1 * 3);
    aris::dynamic::s_pp2pp(real_pm3, leg_in_ground + 2 * 3, xyz_in_leg + 2 * 3);
    aris::dynamic::s_pp2pp(real_pm4, leg_in_ground + 3 * 3, xyz_in_leg + 3 * 3);

    
    leg_12(xyz_in_leg + 0 * 3, input + 0 * 3);//1
    leg_12(xyz_in_leg + 1 * 3, input + 1 * 3);//2
    leg_34(xyz_in_leg + 2 * 3, input + 2 * 3);//3
    leg_34(xyz_in_leg + 3 * 3, input + 3 * 3);//4

    return 0;
}

auto inverseSymmetry(double* leg_in_ground, double* body_in_ground, double* input)->int
{
    double real_pm1[16] = { 0 }, real_pm2[16] = { 0 }, real_pm3[16] = { 0 }, real_pm4[16] = { 0 };
    aris::dynamic::s_pm_dot_inv_pm(PL1, body_in_ground, real_pm1);
    aris::dynamic::s_pm_dot_inv_pm(PL2, body_in_ground, real_pm2);
    aris::dynamic::s_pm_dot_inv_pm(PL3, body_in_ground, real_pm3);
    aris::dynamic::s_pm_dot_inv_pm(PL4, body_in_ground, real_pm4);

    double xyz_in_leg[12] = { 0 }; //腿末端在腿坐标系下的表达
    aris::dynamic::s_pp2pp(real_pm1, leg_in_ground + 0 * 3, xyz_in_leg + 0 * 3);
    aris::dynamic::s_pp2pp(real_pm2, leg_in_ground + 1 * 3, xyz_in_leg + 1 * 3);
    aris::dynamic::s_pp2pp(real_pm3, leg_in_ground + 2 * 3, xyz_in_leg + 2 * 3);
    aris::dynamic::s_pp2pp(real_pm4, leg_in_ground + 3 * 3, xyz_in_leg + 3 * 3);


    leg_12(xyz_in_leg + 0 * 3, input + 0 * 3);//1
    leg_34(xyz_in_leg + 1 * 3, input + 1 * 3);//2
    leg_12(xyz_in_leg + 2 * 3, input + 2 * 3);//3
    leg_34(xyz_in_leg + 3 * 3, input + 3 * 3);//4

    return 0;
}

