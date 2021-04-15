#include"filter.h"
#include <iostream>
#include <stdlib.h>

using namespace std;


// �޷�+�ƶ�ƽ���˲� //
const double limit_amp = 0.1;//���β�����֮��ķ�������
const int window_size = 5;//�ٽ��е����˲�
double value_buf[window_size + 1] = { 0 };

auto filter_limit_average(double input_value, double& output_value)->void
{
    static double pre_value = 0;
    double sum = 0.0;
    // �޷� //
    if (std::abs(input_value - pre_value) > limit_amp)
        value_buf[window_size] = pre_value;
    else
        value_buf[window_size] = input_value;
    

    // �ƶ�ƽ�� //
    for (int i = 0; i < window_size; ++i)
    {
        value_buf[i] = value_buf[i + 1];
        sum += value_buf[i];
    }
    output_value = 1.0 * sum / window_size;

    pre_value = output_value;//���˲����������ݱ���Ϊ��һ������
}



