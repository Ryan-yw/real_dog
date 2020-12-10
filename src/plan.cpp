
#include"plan.h"
#include<cmath>
#include<iostream>
#include"kinematics.h"
using namespace std;


//行走参数静态变量
static double foot_position_start_point[12] = {
										 kBodyLong / 2, 0, -(kBodyWidth / 2) - L1,  //leg1 ->012
										-kBodyLong / 2, 0, -(kBodyWidth / 2) - L1,  //leg2 ->345
										-kBodyLong / 2, 0,  (kBodyWidth / 2) + L1,   //leg3 ->678
										 kBodyLong / 2, 0,  (kBodyWidth / 2) + L1    //leg4 ->91011
};
static double body_position_start_point[16] = { 1,0,0,0,
											   0,1,0,kBodyHigh,
											   0,0,1,0,
											   0,0,0,1 };
static double body_pose_start_pitch = 0;
static double body_pose_start_yaw = 0;
static double body_pose_start_roll = 0;


extern double file_current_leg[12];
extern double file_current_body[12];
/////////////////////////////////////梯形曲线///////////////////////////////////////////////
//生成梯形曲线0->1
//输入：时间，每毫秒计数一次
//输出：当前时刻s的值
auto TCurve::getTCurve(int count)->double
{
	//double ta = p.ta_;
	//double a = p.a_;
	//double v = p.v_;
	//double T_c = p.Tc_;
	int t = count + 1;
	double s = 0;

	if (2 * ta_ == Tc_)   //三角形曲线
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else
		{
			s = 0.5 * a_ * ta_ * ta_ + 0.5 * (t / 1000.0 - ta_) * (2 * v_ - a_ * (t / 1000.0 - ta_));
		}
	}
	else    //梯形曲线
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else if (t >= ta_ * 1000 && t < (Tc_ * 1000 - ta_ * 1000))
		{
			s = v_ * t / 1000 - v_ * v_ / 2.0 / a_;
		}
		else
		{
			s = (2 * a_ * v_ * Tc_ - 2 * v_ * v_ - a_ * a_ * (t / 1000.0 - Tc_) * (t / 1000.0 - Tc_)) / (2 * a_);
		}
	}
	//std::cout << s << std::endl;
	return s;
}

//计算梯形曲线的参数，由成员函数初始化，对应输入参数由构造函数初始化
auto TCurve::getCurveParam()->void
{
	if (v_ * v_ / a_ <= 1)
	{
		this->Tc_ = (a_ + v_ * v_) / v_ / a_;
		this->a_ = a_;
		this->v_ = v_;
	}
	else
	{
		//安速度计算，此时给定的加速度不起作用
		this->Tc_ = 2.0 / v_;
		this->a_ = v_ * v_;
		this->v_ = v_;
	}
	this->ta_ = v_ / a_;
}



//////////////////////////////////////////////////////////////////////////椭圆轨迹////////////////////////////////////////////////

//生成椭圆轨迹，在Tc时间内  x方向0->a;y方向0->b->0;z方向0->c。对应输入参数由构造函数初始化。
auto EllipseTrajectory::getEllipseTrajectory(int count)->void
{
	x_ = a_ * (1 + cos(PI - PI * s_.getTCurve(count))) / 2.0;
	y_ = b_ * sin(PI - PI * s_.getTCurve(count));
	z_ = c_ * (1 + cos(PI - PI * s_.getTCurve(count))) / 2.0;
	//std::cout << y << std::endl;
}



/////////////////////////////////////////////////////////////////////////身体旋转角度轨迹/////////////////////////////////////////////////////////////////////

//生成身体绕xyz三轴旋转的角度轨迹0->theta
auto BodyPose::getBodyRotationTrajectory(int count)->void
{
	pitch_ = pitch_angle_z_ * PI * b_r_s_.getTCurve(count) / 180.0;
	roll_ = roll_angle_x_ * PI * b_r_s_.getTCurve(count) / 180.0;
	yaw_ = yaw_angle_y_ * PI * b_r_s_.getTCurve(count) / 180.0;

	//std::cout << b_r_s_.getTCurve << std::endl;
	//std::cout << pitch_ << std::endl;
}



/////////////////////////////////////////////////脚和身体位置和姿态的规划//////////////////////////////////////////////////////

//对角步态足端在笛卡尔空间下的坐标规划
//当前脚的位置 = 上一步脚的位置 + 脚位置增量
//每当计算完一次梯形曲线，脚的位置跟新一次
//注意：#脚的位置在初始时刻要初始化一下，否则一开始只有脚13动，24脚的初始位置时0
//#注意：目前只适用于平地行走
auto planLegTrot(int e_1, int n, double* current_leg, int count, EllipseTrajectory* Ellipse)->void
{
	if (count == 0)//初始化脚的位置，否则24脚初始位置为0
	{
		for (int i = 0; i < 12; i++)
		{
			current_leg[i] = foot_position_start_point[i];
		}
	}
	Ellipse->getEllipseTrajectory(count);

	if (e_1 % 2 == 0)  //偶数13迈腿，24停
	{
		if (e_1 == 0)   //加速段
		{
			//规划leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;
			//规划leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x() / 2;
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z() / 2;
		}
		else
		{
			//规划leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x();
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z();


			//规划leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x();
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z();
		}
	}
	else if (e_1 % 2 == 1)  //奇数24迈腿13停
	{
		if (e_1 == (2 * n - 1))//减速段
		{
			//规划leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;
			//规划leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x() / 2;
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z() / 2;
		}
		else
		{
			//规划leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x();
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z();
			//规划leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x();
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z();
		}
	}

	if (count + 1 == floor(Ellipse->getTcurve().getTc() * 1000))
	{
		for (int i = 0; i < 12; i++)
		{
			foot_position_start_point[i] = current_leg[i];
		}
	}
}

//静态步态足端在笛卡尔空间下的坐标规划
//当前脚的位置 = 上一步脚的位置 + 脚位置增量
//每当计算完一次梯形曲线，脚的位置跟新一次
//注意：#脚的位置在初始时刻要初始化一下，否则一开始只有脚13动，24脚的初始位置时0
//#注意：目前只适用于平地行走
auto planLegWalk(int e_1, int n, double* current_leg, int count, EllipseTrajectory* Ellipse)->void
{
	if (count == 0)//初始化脚的位置，否则24脚初始位置为0
	{
		for (int i = 0; i < 12; i++)
		{
			current_leg[i] = foot_position_start_point[i];
		}
	}
	Ellipse->getEllipseTrajectory(count);
	//1423
	//if ((e_1 + 1) % 4 == 1)  //迈1腿
	//{
	//	if (e_1 == 0)   //加速段
	//	{
	//		//规划leg1
	//		current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
	//		current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
	//		current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;
	//	}
	//	else
	//	{
	//		//规划leg1
	//		current_leg[0] = foot_position_start_point[0] + Ellipse->get_x();
	//		current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
	//		current_leg[2] = foot_position_start_point[2] + Ellipse->get_z();
	//	}
	//}
	//else if ((e_1 + 1) % 4 == 2)  //迈4腿
	//{
	//	if (e_1 == 1)//加速段
	//	{
	//		//规划leg4
	//		current_leg[9] = foot_position_start_point[9] + Ellipse->get_x() / 2;
	//		current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
	//		current_leg[11] = foot_position_start_point[11] + Ellipse->get_z() / 2;
	//	}
	//	else
	//	{
	//		//规划leg4
	//		current_leg[9] = foot_position_start_point[9] + Ellipse->get_x();
	//		current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
	//		current_leg[11] = foot_position_start_point[11] + Ellipse->get_z();
	//	}
	//}
	//else if ((e_1 + 1) % 4 == 3)  //迈2腿
	//{
	//	if (e_1 == (4 * n - 2))//减速段
	//	{
	//		//规划leg2
	//		current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
	//		current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
	//		current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;
	//	}
	//	else
	//	{
	//		//规划leg2
	//		current_leg[3] = foot_position_start_point[3] + Ellipse->get_x();
	//		current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
	//		current_leg[5] = foot_position_start_point[5] + Ellipse->get_z();
	//	}
	//}
	//else //迈3腿
	//{
	//	if (e_1 == (4 * n - 1))//减速段
	//	{
	//		//规划leg3
	//		current_leg[6] = foot_position_start_point[6] + Ellipse->get_x() / 2;
	//		current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
	//		current_leg[8] = foot_position_start_point[8] + Ellipse->get_z() / 2;
	//	}
	//	else
	//	{
	//		//规划leg3
	//		current_leg[6] = foot_position_start_point[6] + Ellipse->get_x();
	//		current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
	//		current_leg[8] = foot_position_start_point[8] + Ellipse->get_z();
	//	}
	//}
	//1324
	if ((e_1 + 1) % 4 == 1)  //迈1腿
	{
		if (e_1 == 0)   //加速段
		{
			//规划leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;
		}
		else
		{
			//规划leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x();
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z();
		}
	}
	else if ((e_1 + 1) % 4 == 2)  //迈3腿
	{
		if (e_1 == 1)//加速段
		{
			//规划leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x() / 2;
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z() / 2;
		}
		else
		{
			//规划leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x();
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z();
		}
	}
	else if ((e_1 + 1) % 4 == 3)  //迈2腿
	{
		if (e_1 == (4 * n - 2))//减速段
		{
			//规划leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;
		}
		else
		{
			//规划leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x();
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z();
		}
	}
	else //迈4腿
	{
		if (e_1 == (4 * n - 1))//减速段
		{
			//规划leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x() / 2;
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z() / 2;
		}
		else
		{
			//规划leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x();
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z();
		}
	}

	////1432
	//if ((e_1 + 1) % 4 == 1)  //迈1腿
	//{
	//	if (e_1 == 0)   //加速段
	//	{
	//		//规划leg1
	//		current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
	//		current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
	//		current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;
	//	}
	//	else
	//	{
	//		//规划leg1
	//		current_leg[0] = foot_position_start_point[0] + Ellipse->get_x();
	//		current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
	//		current_leg[2] = foot_position_start_point[2] + Ellipse->get_z();
	//	}
	//}
	//else if ((e_1 + 1) % 4 == 2)  //迈4腿
	//{
	//	if (e_1 == 1)//加速段
	//	{
	//		//规划leg4
	//		current_leg[9] = foot_position_start_point[9] + Ellipse->get_x() / 2;
	//		current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
	//		current_leg[11] = foot_position_start_point[11] + Ellipse->get_z() / 2;

	//	}
	//	else
	//	{
	//		//规划leg4
	//		current_leg[9] = foot_position_start_point[9] + Ellipse->get_x();
	//		current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
	//		current_leg[11] = foot_position_start_point[11] + Ellipse->get_z();

	//	}
	//}
	//else if ((e_1 + 1) % 4 == 3)  //迈3腿
	//{
	//	if (e_1 == (4 * n - 2))//减速段
	//	{
	//		//规划leg3
	//		current_leg[6] = foot_position_start_point[6] + Ellipse->get_x() / 2;
	//		current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
	//		current_leg[8] = foot_position_start_point[8] + Ellipse->get_z() / 2;
	//	}
	//	else
	//	{
	//		//规划leg3
	//		current_leg[6] = foot_position_start_point[6] + Ellipse->get_x();
	//		current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
	//		current_leg[8] = foot_position_start_point[8] + Ellipse->get_z();
	//	}
	//}
	//else //迈4腿
	//{
	//	if (e_1 == (4 * n - 1))//减速段
	//	{
	//		//规划leg2
	//		current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
	//		current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
	//		current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;
	//	}
	//	else
	//	{
	//		//规划leg2
	//		current_leg[3] = foot_position_start_point[3] + Ellipse->get_x();
	//		current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
	//		current_leg[5] = foot_position_start_point[5] + Ellipse->get_z();
	//	}
	//}

	if (count == floor(Ellipse->getTcurve().getTc() * 1000) - 1)
	{
		for (int i = 0; i < 12; i++)
		{
			foot_position_start_point[i] = current_leg[i];
		}
	}
}


//本函数用于规划四足机器人在对角步态下身体的位置轨迹，不加旋转（姿态变换）
//当前身体的位置 = 上一步身体的位置 + 身体位置增量
//每当结束一次命令是，身体的位置跟新一次
//#注意：目前只适用于平地行走
auto planBodyTransformTrot(int e_1, int n, double* current_body, int count, EllipseTrajectory* Ellipse)->void
{
	int per_step_count = Ellipse->getTcurve().getTc() * 1000;
	if (count == 0) //有用，不能删，否则算不出角度
	{
		for (int i = 0; i < 16; i++)
		{
			current_body[i] = body_position_start_point[i];
		}
	}

	if (e_1 == 0)   //加速段
	{
		//规划身体
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() * count * count / (4.0 * per_step_count * per_step_count);
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() * count * count / (4.0 * per_step_count * per_step_count);
	}
	else if (e_1 == (2 * n - 1))//减速段
	{

		//规划身体
		int t = (2 * n - 1) * per_step_count + per_step_count;
		current_body[3] = body_position_start_point[3] + 0 - Ellipse->get_a() * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + Ellipse->get_a() * n - Ellipse->get_a() / 2.0;//n * a
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + 0 - Ellipse->get_c() * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + Ellipse->get_c() * n - Ellipse->get_c() / 2.0;
	}
	else //匀速段
	{
		//规划身体
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() / 4.0 + Ellipse->get_a() * (count - per_step_count) / per_step_count / 2;//速度为100mm/s  每秒计数per_step_count
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() / 4.0 + Ellipse->get_c() * (count - per_step_count) / per_step_count / 2;
	}

	if (count + 1 >= 2 * n * per_step_count)
	{
		for (int i = 0; i < 16; i++)
		{
			body_position_start_point[i] = current_body[i];
		}
	}
}

//本函数用于规划四足机器人在walk步态下身体的位置轨迹，不加旋转（姿态变换）
//当前身体的位置 = 上一步身体的位置 + 身体位置增量
//每当结束一次命令是，身体的位置跟新一次
//#注意：目前只适用于平地行走
auto planBodyTransformWalk(int e_1, int n, double* current_body, int count, EllipseTrajectory* Ellipse)->void
{
	int per_step_count = Ellipse->getTcurve().getTc() * 1000;
	if (count == 0) //有用，不能删，否则算不出角度
	{
		for (int i = 0; i < 16; i++)
		{
			current_body[i] = body_position_start_point[i];
		}
	}

	if (e_1 == 0 || e_1 == 1)   //加速段
	{
		//规划身体
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() * count * count / (4.0 * 2 * per_step_count * 2 * per_step_count);
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() * count * count / (4.0 * 2 * per_step_count * 2 * per_step_count);
	}
	else if (e_1 == (4 * n - 1) || e_1 == (4 * n - 2))//减速段
	{

		//规划身体
		int t = 4 * n * per_step_count;
		current_body[3] = body_position_start_point[3] + 0 - Ellipse->get_a() * (count - t) * (count - t) / (4.0 * 2*per_step_count * 2*per_step_count) + Ellipse->get_a() * n - Ellipse->get_a() / 2.0;//n * a 
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + 0 - Ellipse->get_c() * (count - t) * (count - t) / (4.0 * 2*per_step_count * 2*per_step_count) + Ellipse->get_c() * n - Ellipse->get_c() / 2.0;
	}
	else //匀速段
	{
		//规划身体，在加速段的基础上计算。斜率可自定
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() / 4.0 + Ellipse->get_a() * (count - 2 * per_step_count) / per_step_count / 4.0;//速度为100mm/s  每秒计数per_step_count
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() / 4.0 + Ellipse->get_c() * (count - 2 * per_step_count) / per_step_count / 4.0;
	}

	if (count + 1 >= 4 * n * per_step_count)
	{
		for (int i = 0; i < 16; i++)
		{
			body_position_start_point[i] = current_body[i];
		}
	}
}


//本函数用于规划四足机器人身体的姿态变换
//#注意：只适用于当脚不动时，身体扭动的情形
//可能会出现的问题   body_pisiton_start_point*R_z
auto planBodyRotation(int count, double* current_body, BodyPose* body_pose_param)->void
{
	static double pitch = 0;
	static double roll = 0;
	static double yaw = 0;
	//开始时读取之前的值
	if (count == 0)
	{
		pitch = body_pose_start_pitch;
		roll = body_pose_start_roll;
		yaw = body_pose_start_yaw;
	}

	body_pose_param->getBodyRotationTrajectory(count);

	pitch = body_pose_start_pitch + body_pose_param->getCurrentPitch();
	roll = body_pose_start_roll + body_pose_param->getCurrentRoll();
	yaw = body_pose_start_yaw + body_pose_param->getCurrentYaw();

	double R_x[16] = {
					  1,         0,          0, 0,
					  0, std::cos(roll), -std::sin(roll), 0,
					  0, std::sin(roll),  std::cos(roll), 0,
					  0,         0,          0, 1
	};
	double R_y[16] = {
						 std::cos(yaw), 0, std::sin(yaw), 0,
								0, 1,        0, 0,
						-std::sin(yaw), 0, std::cos(yaw), 0,
								0, 0,        0, 1
	};
	double R_z[16] = {
						std::cos(pitch), -std::sin(pitch), 0, 0,
						std::sin(pitch),  std::cos(pitch), 0, 0,
								 0,           0, 1, 0,
								 0,           0, 0, 1
	};


	double tempx[16] = { 0 }; double tempy[16] = { 0 }; double tempz[16] = { 0 };




	if (body_pose_param->getPitchTotalAngle() != 0 && body_pose_param->getRollTotalAngle() == 0 && body_pose_param->getYawTotalAngle() == 0) //pitch
	{
		s_pm_dot_pm(R_z, body_position_start_point, tempz);
		std::copy(tempz, tempz + 16, current_body);
	}
	else if (body_pose_param->getPitchTotalAngle() == 0 && body_pose_param->getRollTotalAngle() != 0 && body_pose_param->getYawTotalAngle() == 0) //roll
	{
		s_pm_dot_pm(body_position_start_point, R_x, tempx);
		std::copy(tempx, tempx + 16, current_body);
	}
	else //yaw
	{
		s_pm_dot_pm(body_position_start_point, R_y, tempy);
		std::copy(tempy, tempy + 16, current_body);
	}
	//结束时保存变化之后的值
	if (count + 1 == floor(body_pose_param->getTcurve().getTc() * 1000))
	{
		body_pose_start_pitch = pitch;
		body_pose_start_roll = roll;
		body_pose_start_yaw = yaw;
	}
}


//本函数用于规划四足机器人下蹲和起立  
//#注意：只适用于当脚不动时，身体上下动的情况
auto planBodyUpDown(int count, double* current_body, EllipseTrajectory* body_p_param,double distance)->void
{

	//开始时读取之前的值
	if (count == 0) //有用，不能删，否则算不出角度
	{
		for (int i = 0; i < 16; i++)
		{
			current_body[i] = body_position_start_point[i];
		}
	}

	current_body[7] = body_position_start_point[7] + distance * body_p_param->getTcurve().getTCurve(count);

	//结束时保存变化之后的值
	if (count + 1 == floor(body_p_param->getTcurve().getTc() * 1000))
	{
		for (int i = 0; i < 16; i++)
		{
			body_position_start_point[i] = current_body[i];
		}
	}
}


/////////////////////////////////////////////////////////////////步态规划/////////////////////////////////////////////////////////////

//以下函数在robot.cpp中被调用


//机器人行走对角步态，包括原地踏步、前进、后退、左移、右移。
//其中步长步高和步数可由用户输入。走一步的时间（或行走快慢）可由用户输入梯形曲线的速度和加速度确定
//#注意：行走最大速度和加速度还没测试
auto trotPlanSameLeg(int n, int count, EllipseTrajectory* Ellipse, double* input)->int
{

	int per_step_count = Ellipse->getTcurve().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };


	//判断行走状态
	int e_1 = count / per_step_count;  //判断当前在走哪一步,腿走一步e1加1

	//规划腿
	planLegTrot(e_1, n, current_leg_in_ground, count % per_step_count, Ellipse);
	//规划身体
	planBodyTransformTrot(e_1, n, current_body_in_ground, count, Ellipse);

	//模型测试使用
	for (int j = 0; j < 12; j++)
	{
		file_current_leg[j] = current_leg_in_ground[j];
	}
	for (int j = 0; j < 12; j++)
	{
		file_current_body[j] = current_body_in_ground[j];
	}
	//模型测试使用
	inverseSame(current_leg_in_ground, current_body_in_ground, input);

	return 2 * n * per_step_count - count - 1;
}

//机器人行走对角步态，包括原地踏步、前进、后退、左移、右移。
//其中步长步高和步数可由用户输入。走一步的时间（或行走快慢）可由用户输入梯形曲线的速度和加速度确定
//#注意：行走最大速度和加速度还没测试
auto trotPlanSymmetryLeg(int n, int count, EllipseTrajectory* Ellipse, double* input)->int
{

	int per_step_count = Ellipse->getTcurve().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };


	//判断行走状态
	int e_1 = count / per_step_count;  //判断当前在走哪一步,腿走一步e1加1

	//规划腿
	planLegTrot(e_1, n, current_leg_in_ground, count % per_step_count, Ellipse);
	//规划身体
	planBodyTransformTrot(e_1, n, current_body_in_ground, count, Ellipse);

	//模型测试使用
	for (int j = 0; j < 12; j++)
	{
		file_current_leg[j] = current_leg_in_ground[j];
	}
	for (int j = 0; j < 12; j++)
	{
		file_current_body[j] = current_body_in_ground[j];
	}
	//模型测试使用
	inverseSymmetry(current_leg_in_ground, current_body_in_ground, input);

	return 2 * n * per_step_count - count - 1;
}

//机器人行走静态步态，包括原地踏步、前进、后退、左移、右移。适用于四条腿初始姿态一样的情况
//其中步长步高和步数可由用户输入。走一步的时间（或行走快慢）可由用户输入梯形曲线的速度和加速度确定
//#注意：行走最大速度和加速度还没测试
auto walkPlanSameLeg(int n, int count, EllipseTrajectory* Ellipse, double* input)->int
{

	int per_step_count = Ellipse->getTcurve().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };


	//判断行走状态
	int e_1 = count / per_step_count;  //判断当前在走哪一步,腿走一步e1加1

	//规划腿
	planLegWalk(e_1, n, current_leg_in_ground, count % per_step_count, Ellipse);
	//规划身体
	planBodyTransformWalk(e_1, n, current_body_in_ground, count, Ellipse);
	//模型测试使用
	for (int j = 0; j < 12; j++)
	{
		file_current_leg[j] = current_leg_in_ground[j];
	}
	for (int j = 0; j < 12; j++)
	{
		file_current_body[j] = current_body_in_ground[j];
	}
	//模型测试使用
	inverseSame(current_leg_in_ground, current_body_in_ground, input);

	return 4 * n * per_step_count - count - 1;
}

//机器人行走静态步态，包括原地踏步、前进、后退、左移、右移。适用于前后腿对称的情况
//其中步长步高和步数可由用户输入。走一步的时间（或行走快慢）可由用户输入梯形曲线的速度和加速度确定
//#注意：行走最大速度和加速度还没测试
auto walkPlanSymmetryLeg(int n, int count, EllipseTrajectory* Ellipse, double* input)->int
{

	int per_step_count = Ellipse->getTcurve().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };


	//判断行走状态
	int e_1 = count / per_step_count;  //判断当前在走哪一步,腿走一步e1加1

	//规划腿
	planLegWalk(e_1, n, current_leg_in_ground, count % per_step_count, Ellipse);
	//规划身体
	planBodyTransformWalk(e_1, n, current_body_in_ground, count, Ellipse);
	//模型测试使用
	for (int j = 0; j < 12; j++)
	{
		file_current_leg[j] = current_leg_in_ground[j];
	}
	for (int j = 0; j < 12; j++)
	{
		file_current_body[j] = current_body_in_ground[j];
	}
	//模型测试使用
	inverseSymmetry(current_leg_in_ground, current_body_in_ground, input);

	return 4 * n * per_step_count - count - 1;
}


//机器人原地扭动步态，包括原地俯仰，横滚，偏航
//#注意：只能完成单独的一项，比如要实现先俯仰后偏航，必须等俯仰结束后恢复到初始位置才能进行偏航
auto posePlan(int count, EllipseTrajectory* Ellipse, BodyPose* body_pose_param, double* input)->int
{

	int per_step_count = Ellipse->getTcurve().getTc() * 1000;


	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };


	//std::cout << current_body_in_ground[3] << std::endl;
	//std::cout << body_pose_param->pitch_ << std::endl;

	//规划腿
	planLegTrot(0, 1, current_leg_in_ground, count % per_step_count, Ellipse);
	//规划身体姿态
	planBodyRotation(count, current_body_in_ground, body_pose_param);
	//模型测试使用
	for (int j = 0; j < 12; j++)
	{
		file_current_leg[j] = current_leg_in_ground[j];
	}
	for (int j = 0; j < 12; j++)
	{
		file_current_body[j] = current_body_in_ground[j];
	}
	//模型测试使用

	inverseSame(current_leg_in_ground, current_body_in_ground, input);

	return  per_step_count - count - 1;
}

auto updownPlanSame(int count, EllipseTrajectory* Ellipse,double distance, double* input)->int
{

	int per_step_count = Ellipse->getTcurve().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };

	//规划腿
	planLegTrot(0, 1, current_leg_in_ground, count % per_step_count, Ellipse);
	//规划身体
	planBodyUpDown(count, current_body_in_ground, Ellipse,distance);

	//模型测试使用
	for (int j = 0; j < 12; j++)
	{
		file_current_leg[j] = current_leg_in_ground[j];
	}
	for (int j = 0; j < 12; j++)
	{
		file_current_body[j] = current_body_in_ground[j];
	}
	//模型测试使用
	inverseSame(current_leg_in_ground, current_body_in_ground, input);


	return  per_step_count - count - 1;
}

auto updownPlanSymmetry(int count, EllipseTrajectory* Ellipse, double distance, double* input)->int
{

	int per_step_count = Ellipse->getTcurve().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };

	//规划腿
	planLegTrot(0, 1, current_leg_in_ground, count % per_step_count, Ellipse);
	//规划身体
	planBodyUpDown(count, current_body_in_ground, Ellipse, distance);

	//模型测试使用
	for (int j = 0; j < 12; j++)
	{
		file_current_leg[j] = current_leg_in_ground[j];
	}
	for (int j = 0; j < 12; j++)
	{
		file_current_body[j] = current_body_in_ground[j];
	}
	//模型测试使用
	inverseSymmetry(current_leg_in_ground, current_body_in_ground, input);


	return  per_step_count - count - 1;
}
