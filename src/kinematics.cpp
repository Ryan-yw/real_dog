#include"kinematics.h"
#include<cmath>
#include<iostream>
#include<fstream>


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

//矩阵计算
void s_inv_pm(const double* pm_in, double* pm_out)
{
    //转置
    pm_out[0] = pm_in[0];
    pm_out[1] = pm_in[4];
    pm_out[2] = pm_in[8];
    pm_out[4] = pm_in[1];
    pm_out[5] = pm_in[5];
    pm_out[6] = pm_in[9];
    pm_out[8] = pm_in[2];
    pm_out[9] = pm_in[6];
    pm_out[10] = pm_in[10];

    //位置
    pm_out[3] = -pm_out[0] * pm_in[3] - pm_out[1] * pm_in[7] - pm_out[2] * pm_in[11];
    pm_out[7] = -pm_out[4] * pm_in[3] - pm_out[5] * pm_in[7] - pm_out[6] * pm_in[11];
    pm_out[11] = -pm_out[8] * pm_in[3] - pm_out[9] * pm_in[7] - pm_out[10] * pm_in[11];

    //其他
    pm_out[12] = 0;
    pm_out[13] = 0;
    pm_out[14] = 0;
    pm_out[15] = 1;
}
double* s_pm_dot_pm(const double* pm1, const double* pm2, double* pm_out)
{
    pm_out[0] = pm1[0] * pm2[0] + pm1[1] * pm2[4] + pm1[2] * pm2[8];
    pm_out[1] = pm1[0] * pm2[1] + pm1[1] * pm2[5] + pm1[2] * pm2[9];
    pm_out[2] = pm1[0] * pm2[2] + pm1[1] * pm2[6] + pm1[2] * pm2[10];
    pm_out[3] = pm1[0] * pm2[3] + pm1[1] * pm2[7] + pm1[2] * pm2[11] + pm1[3];

    pm_out[4] = pm1[4] * pm2[0] + pm1[5] * pm2[4] + pm1[6] * pm2[8];
    pm_out[5] = pm1[4] * pm2[1] + pm1[5] * pm2[5] + pm1[6] * pm2[9];
    pm_out[6] = pm1[4] * pm2[2] + pm1[5] * pm2[6] + pm1[6] * pm2[10];
    pm_out[7] = pm1[4] * pm2[3] + pm1[5] * pm2[7] + pm1[6] * pm2[11] + pm1[7];

    pm_out[8] = pm1[8] * pm2[0] + pm1[9] * pm2[4] + pm1[10] * pm2[8];
    pm_out[9] = pm1[8] * pm2[1] + pm1[9] * pm2[5] + pm1[10] * pm2[9];
    pm_out[10] = pm1[8] * pm2[2] + pm1[9] * pm2[6] + pm1[10] * pm2[10];
    pm_out[11] = pm1[8] * pm2[3] + pm1[9] * pm2[7] + pm1[10] * pm2[11] + pm1[11];

    pm_out[12] = 0;
    pm_out[13] = 0;
    pm_out[14] = 0;
    pm_out[15] = 1;

    return pm_out;
}
double* s_inv_pm_dot_pm(const double* inv_pm, const double* pm, double* pm_out)
{
    pm_out[0] = inv_pm[0] * pm[0] + inv_pm[4] * pm[4] + inv_pm[8] * pm[8];
    pm_out[1] = inv_pm[0] * pm[1] + inv_pm[4] * pm[5] + inv_pm[8] * pm[9];
    pm_out[2] = inv_pm[0] * pm[2] + inv_pm[4] * pm[6] + inv_pm[8] * pm[10];
    pm_out[3] = inv_pm[0] * (pm[3] - inv_pm[3]) + inv_pm[4] * (pm[7] - inv_pm[7]) + inv_pm[8] * (pm[11] - inv_pm[11]);

    pm_out[4] = inv_pm[1] * pm[0] + inv_pm[5] * pm[4] + inv_pm[9] * pm[8];
    pm_out[5] = inv_pm[1] * pm[1] + inv_pm[5] * pm[5] + inv_pm[9] * pm[9];
    pm_out[6] = inv_pm[1] * pm[2] + inv_pm[5] * pm[6] + inv_pm[9] * pm[10];
    pm_out[7] = inv_pm[1] * (pm[3] - inv_pm[3]) + inv_pm[5] * (pm[7] - inv_pm[7]) + inv_pm[9] * (pm[11] - inv_pm[11]);

    pm_out[8] = inv_pm[2] * pm[0] + inv_pm[6] * pm[4] + inv_pm[10] * pm[8];
    pm_out[9] = inv_pm[2] * pm[1] + inv_pm[6] * pm[5] + inv_pm[10] * pm[9];
    pm_out[10] = inv_pm[2] * pm[2] + inv_pm[6] * pm[6] + inv_pm[10] * pm[10];
    pm_out[11] = inv_pm[2] * (pm[3] - inv_pm[3]) + inv_pm[6] * (pm[7] - inv_pm[7]) + inv_pm[10] * (pm[11] - inv_pm[11]);

    pm_out[12] = 0;
    pm_out[13] = 0;
    pm_out[14] = 0;
    pm_out[15] = 1;

    return pm_out;
}
double* s_pm_dot_inv_pm(const double* pm, const double* inv_pm, double* pm_out)
{
    pm_out[0] = pm[0] * inv_pm[0] + pm[1] * inv_pm[1] + pm[2] * inv_pm[2];
    pm_out[1] = pm[0] * inv_pm[4] + pm[1] * inv_pm[5] + pm[2] * inv_pm[6];
    pm_out[2] = pm[0] * inv_pm[8] + pm[1] * inv_pm[9] + pm[2] * inv_pm[10];
    pm_out[3] = -pm_out[0] * inv_pm[3] - pm_out[1] * inv_pm[7] - pm_out[2] * inv_pm[11] + pm[3];

    pm_out[4] = pm[4] * inv_pm[0] + pm[5] * inv_pm[1] + pm[6] * inv_pm[2];
    pm_out[5] = pm[4] * inv_pm[4] + pm[5] * inv_pm[5] + pm[6] * inv_pm[6];
    pm_out[6] = pm[4] * inv_pm[8] + pm[5] * inv_pm[9] + pm[6] * inv_pm[10];
    pm_out[7] = -pm_out[4] * inv_pm[3] - pm_out[5] * inv_pm[7] - pm_out[6] * inv_pm[11] + pm[7];

    pm_out[8] = pm[8] * inv_pm[0] + pm[9] * inv_pm[1] + pm[10] * inv_pm[2];
    pm_out[9] = pm[8] * inv_pm[4] + pm[9] * inv_pm[5] + pm[10] * inv_pm[6];
    pm_out[10] = pm[8] * inv_pm[8] + pm[9] * inv_pm[9] + pm[10] * inv_pm[10];
    pm_out[11] = -pm_out[8] * inv_pm[3] - pm_out[9] * inv_pm[7] - pm_out[10] * inv_pm[11] + pm[11];

    pm_out[12] = 0;
    pm_out[13] = 0;
    pm_out[14] = 0;
    pm_out[15] = 1;

    return pm_out;
}
double* s_pm_dot_v3(const double* pm, const double* v3, double* v3_out)
{
    v3_out[0] = pm[0] * v3[0] + pm[1] * v3[1] + pm[2] * v3[2];
    v3_out[1] = pm[4] * v3[0] + pm[5] * v3[1] + pm[6] * v3[2];
    v3_out[2] = pm[8] * v3[0] + pm[9] * v3[1] + pm[10] * v3[2];

    return v3_out;
}
double* s_inv_pm_dot_v3(const double* inv_pm, const double* v3, double* v3_out)
{
    v3_out[0] = inv_pm[0] * v3[0] + inv_pm[4] * v3[1] + inv_pm[8] * v3[2];
    v3_out[1] = inv_pm[1] * v3[0] + inv_pm[5] * v3[1] + inv_pm[9] * v3[2];
    v3_out[2] = inv_pm[2] * v3[0] + inv_pm[6] * v3[1] + inv_pm[10] * v3[2];

    return v3_out;
}
double* s_pp2pp(const double* relative_pm, const double* from_pp, double* to_pp)
{
    to_pp[0] = relative_pm[0] * from_pp[0] + relative_pm[1] * from_pp[1] + relative_pm[2] * from_pp[2] + relative_pm[3];
    to_pp[1] = relative_pm[4] * from_pp[0] + relative_pm[5] * from_pp[1] + relative_pm[6] * from_pp[2] + relative_pm[7];
    to_pp[2] = relative_pm[8] * from_pp[0] + relative_pm[9] * from_pp[1] + relative_pm[10] * from_pp[2] + relative_pm[11];

    return to_pp;
}
double* s_inv_pp2pp(const double* inv_relative_pm, const double* from_pp, double* to_pp)
{
    double tem[3] = { from_pp[0] - inv_relative_pm[3] ,from_pp[1] - inv_relative_pm[7] ,from_pp[2] - inv_relative_pm[11] };

    to_pp[0] = inv_relative_pm[0] * tem[0] + inv_relative_pm[4] * tem[1] + inv_relative_pm[8] * tem[2];
    to_pp[1] = inv_relative_pm[1] * tem[0] + inv_relative_pm[5] * tem[1] + inv_relative_pm[9] * tem[2];
    to_pp[2] = inv_relative_pm[2] * tem[0] + inv_relative_pm[6] * tem[1] + inv_relative_pm[10] * tem[2];

    return to_pp;
}

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

    if (y <= 0 && z > 0)
    {
        mot_pos_3[0] = A - B;
    }
    else if (y <= 0 && z < 0)
    {
        mot_pos_3[0] = 180 - A - B;
    }
    else if (y > 0 && z >= 0)
    {
        mot_pos_3[0] = A + B;
    }
    else if (y > 0 && z <= 0)
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

    if (y <= 0 && z > 0)
    {
        mot_pos_3[0] = A - B;
    }
    else if (y <= 0 && z < 0)
    {
        mot_pos_3[0] = 180 - A - B;
    }
    else if (y > 0 && z >= 0)
    {
        mot_pos_3[0] = A + B;
    }
    else if (y > 0 && z <= 0)
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
    s_pm_dot_inv_pm(PL1, body_in_ground, real_pm1);
    s_pm_dot_inv_pm(PL2, body_in_ground, real_pm2);
    s_pm_dot_inv_pm(PL3, body_in_ground, real_pm3);
    s_pm_dot_inv_pm(PL4, body_in_ground, real_pm4);

    double xyz_in_leg[12] = { 0 }; //腿末端在腿坐标系下的表达
    s_pp2pp(real_pm1, leg_in_ground + 0 * 3, xyz_in_leg + 0 * 3);
    s_pp2pp(real_pm2, leg_in_ground + 1 * 3, xyz_in_leg + 1 * 3);
    s_pp2pp(real_pm3, leg_in_ground + 2 * 3, xyz_in_leg + 2 * 3);
    s_pp2pp(real_pm4, leg_in_ground + 3 * 3, xyz_in_leg + 3 * 3);

    
    leg_12(xyz_in_leg + 0 * 3, input + 0 * 3);//1
    leg_12(xyz_in_leg + 1 * 3, input + 1 * 3);//2
    leg_34(xyz_in_leg + 2 * 3, input + 2 * 3);//3
    leg_34(xyz_in_leg + 3 * 3, input + 3 * 3);//4

    return 0;
}

auto inverseSymmetry(double* leg_in_ground, double* body_in_ground, double* input)->int
{
    double real_pm1[16] = { 0 }, real_pm2[16] = { 0 }, real_pm3[16] = { 0 }, real_pm4[16] = { 0 };
    s_pm_dot_inv_pm(PL1, body_in_ground, real_pm1);
    s_pm_dot_inv_pm(PL2, body_in_ground, real_pm2);
    s_pm_dot_inv_pm(PL3, body_in_ground, real_pm3);
    s_pm_dot_inv_pm(PL4, body_in_ground, real_pm4);

    double xyz_in_leg[12] = { 0 }; //腿末端在腿坐标系下的表达
    s_pp2pp(real_pm1, leg_in_ground + 0 * 3, xyz_in_leg + 0 * 3);
    s_pp2pp(real_pm2, leg_in_ground + 1 * 3, xyz_in_leg + 1 * 3);
    s_pp2pp(real_pm3, leg_in_ground + 2 * 3, xyz_in_leg + 2 * 3);
    s_pp2pp(real_pm4, leg_in_ground + 3 * 3, xyz_in_leg + 3 * 3);


    leg_12(xyz_in_leg + 0 * 3, input + 0 * 3);//1
    leg_34(xyz_in_leg + 1 * 3, input + 1 * 3);//2
    leg_12(xyz_in_leg + 2 * 3, input + 2 * 3);//3
    leg_34(xyz_in_leg + 3 * 3, input + 3 * 3);//4

    return 0;
}

