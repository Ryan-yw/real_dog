#include"filter.h"
#include <iostream>
#include <stdlib.h>

using namespace std;


// 限幅+移动平均滤波 //
const double limit_amp = 0.1;//两次采样点之间的幅度限制
const int window_size = 5;//再进行递推滤波
double value_buf[window_size + 1] = { 0 };

auto filter_limit_average(double input_value, double& output_value)->void
{
    static double pre_value = 0;
    double sum = 0.0;
    // 限幅 //
    if (std::abs(input_value - pre_value) > limit_amp)
        value_buf[window_size] = pre_value;
    else
        value_buf[window_size] = input_value;
    

    // 移动平均 //
    for (int i = 0; i < window_size; ++i)
    {
        value_buf[i] = value_buf[i + 1];
        sum += value_buf[i];
    }
    output_value = 1.0 * sum / window_size;

    pre_value = output_value;//将滤波后的输出数据保存为上一次数据
}



