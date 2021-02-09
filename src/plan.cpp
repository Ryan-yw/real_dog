
#include<cmath>
#include<iostream>
#include<aris.hpp>

#include"kinematics.h"
#include"plan.h"
using namespace std;



static double body_pose_start_pitch = 0;
static double body_pose_start_yaw = 0;
static double body_pose_start_roll = 0;


extern double file_current_leg[12];
extern double file_current_body[16];
//-------------------------------------------------------��������----------------------------------------------------//

//������������0->1
//���룺ʱ�䣬ÿ�������һ��
//�������ǰʱ��s��ֵ
auto TCurve::getTCurve(int count)->double
{
	//double ta = p.ta_;
	//double a = p.a_;
	//double v = p.v_;
	//double T_c = p.Tc_;
	int t = count + 1;
	double s = 0;

	if (2 * ta_ == Tc_)   //����������
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
	else    //��������
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

//�����������ߵĲ������ɳ�Ա������ʼ������Ӧ��������ɹ��캯����ʼ��
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
		//���ٶȼ��㣬��ʱ�����ļ��ٶȲ�������
		this->Tc_ = 2.0 / v_;
		this->a_ = v_ * v_;
		this->v_ = v_;
	}
	this->ta_ = v_ / a_;
}



//---------------------------------------------------------��Բ�켣---------------------------------------------------//

//������Բ�켣����Tcʱ����  x����0->a;y����0->b->0;z����0->c����Ӧ��������ɹ��캯����ʼ����
//��������Ϊ��Բ�ĳ���Ͷ���
auto EllipseTrajectory::getEllipseTrajectory(int count)->void
{
	x_ = a_ * (1 + cos(PI - PI * s_.getTCurve(count))) / 2.0;
	y_ = b_ * sin(PI - PI * s_.getTCurve(count));
	z_ = c_ * (1 + cos(PI - PI * s_.getTCurve(count))) / 2.0;
	//std::cout << y << std::endl;
}



//-----------------------------------------------------������ת�Ƕȹ켣----------------------------------------------//

//����������xyz������ת�ĽǶȹ켣0->theta
auto BodyPose::getBodyRotationTrajectory(int count)->void
{
	pitch_ = pitch_angle_z_ * PI * b_r_s_.getTCurve(count) / 180.0;
	roll_ = roll_angle_x_ * PI * b_r_s_.getTCurve(count) / 180.0;
	yaw_ = yaw_angle_y_ * PI * b_r_s_.getTCurve(count) / 180.0;

	//std::cout << b_r_s_.getTCurve << std::endl;
	//std::cout << pitch_ << std::endl;
}



//---------------------------------------------------�滮��---------------------------------------------//

///����ڵѿ�������ϵ�µĹ滮��ûִ����һ���������߼�¼һ������
///count=e_2 ��0 �� Tc ѭ��  
///�жϵ�ǰ������һ��,����һ��e1��1


//�Խǲ�̬
//��ǰ�ŵ�λ�� = ��һ���ŵ�λ�� + ��λ������
//#ע�⣺Ŀǰֻ������ƽ������
auto planLegTrot(int e_1, int n, double* current_leg, int count, EllipseTrajectory* Ellipse)->void
{
	if (count == 0)//��ʼ���ŵ�λ�ã�����24�ų�ʼλ��Ϊ0
	{
		for (int i = 0; i < 12; ++i)
		{
			current_leg[i] = foot_position_start_point[i];
		}
	}

	Ellipse->getEllipseTrajectory(count);

	if (e_1 % 2 == 0)  //ż��13���ȣ�24ͣ
	{
		if (e_1 == 0)   //���ٶ�
		{
			//�滮leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;
			//�滮leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x() / 2;
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z() / 2;
		}
		else
		{
			//�滮leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x();
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z();


			//�滮leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x();
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z();
		}
	}
	else if (e_1 % 2 == 1)  //����24����13ͣ
	{
		if (e_1 == (2 * n - 1))//���ٶ�
		{
			//�滮leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;
			//�滮leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x() / 2;
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z() / 2;
		}
		else
		{
			//�滮leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x();
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z();
			//�滮leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x();
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z();
		}
	}

	if (count + 1 == floor(Ellipse->get_s().getTc() * 1000))
	{
		for (int i = 0; i < 12; ++i)
		{
			foot_position_start_point[i] = current_leg[i];
		}
	}
}

//��̬��̬
//��ǰ�ŵ�λ�� = ��һ���ŵ�λ�� + ��λ������
//ÿ��������һ���������ߣ��ŵ�λ�ø���һ��
//#ע�⣺Ŀǰֻ������ƽ������
auto planLegWalk(int e_1, int n, double* current_leg, int count, EllipseTrajectory* Ellipse)->void
{
	if (count == 0)//��ʼ���ŵ�λ�ã�����24�ų�ʼλ��Ϊ0
	{
		for (int i = 0; i < 12; ++i)
		{
			current_leg[i] = foot_position_start_point[i];
		}
	}
	Ellipse->getEllipseTrajectory(count);


	//1324
	if ((e_1 + 1) % 4 == 1)  //��1��
	{
		if (e_1 == 0)   //���ٶ�
		{
			//�滮leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;
		}
		else
		{
			//�滮leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x();
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z();
		}
	}
	else if ((e_1 + 1) % 4 == 2)  //��3��
	{
		if (e_1 == 1)//���ٶ�
		{
			//�滮leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x() / 2;
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z() / 2;
		}
		else
		{
			//�滮leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x();
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z();
		}
	}
	else if ((e_1 + 1) % 4 == 3)  //��2��
	{
		if (e_1 == (4 * n - 2))//���ٶ�
		{
			//�滮leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;
		}
		else
		{
			//�滮leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x();
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z();
		}
	}
	else //��4��
	{
		if (e_1 == (4 * n - 1))//���ٶ�
		{
			//�滮leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x() / 2;
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z() / 2;
		}
		else
		{
			//�滮leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x();
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z();
		}
	}



	if (count == floor(Ellipse->get_s().getTc() * 1000) - 1)
	{
		for (int i = 0; i < 12; ++i)
		{
			foot_position_start_point[i] = current_leg[i];
		}
	}
}

//�Խǲ�̬��ԭ����ת
//�滮����ֵ����̧��Ȼ�����ת����
//#ע�⣺Ŀǰֻ������ƽ��
auto planLegTrotTurn(int e_1, double* current_leg, int count, EllipseTrajectory* Ellipse, BodyPose* body_pose_param)->void
{

	//count ��0 �� Tc ѭ��  //�жϵ�ǰ������һ��,����һ��e1��1

	if (count == 0)//��ʼ���ŵ�λ�ã�����24�ų�ʼλ��Ϊ0
	{
		for (int i = 0; i < 12; ++i)
		{
			current_leg[i] = foot_position_start_point[i];
		}
	}
	Ellipse->getEllipseTrajectory(count);

	double temp_xyz_in_ground[12] = { 0 };
	static double yaw = 0;
	//ÿ���������߿�ʼʱ��ȡ֮ǰ��ֵ
	if (count == 0)
	{
		yaw = 0;
	}

	body_pose_param->getBodyRotationTrajectory(count);
	yaw = body_pose_param->getCurrentYaw();

	double R_y[16] = {
						 std::cos(yaw), 0, std::sin(yaw), 0,
								0, 1,        0, 0,
						-std::sin(yaw), 0, std::cos(yaw), 0,
								0, 0,        0, 1
	};

	//�������滮��������̧��
	if (e_1 % 2 == 0)  //ż��13���ȣ�24ͣ
	{
		//�滮leg1
		temp_xyz_in_ground[0] = foot_position_start_point[0] + Ellipse->get_x();
		temp_xyz_in_ground[1] = foot_position_start_point[1] + Ellipse->get_y();
		temp_xyz_in_ground[2] = foot_position_start_point[2] + Ellipse->get_z();


		//�滮leg3
		temp_xyz_in_ground[6] = foot_position_start_point[6] + Ellipse->get_x();
		temp_xyz_in_ground[7] = foot_position_start_point[7] + Ellipse->get_y();
		temp_xyz_in_ground[8] = foot_position_start_point[8] + Ellipse->get_z();

		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 0 * 3, current_leg + 0 * 3);
		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 2 * 3, current_leg + 2 * 3);
	}
	else if (e_1 % 2 == 1)  //����24����13ͣ
	{
		//�滮leg2
		temp_xyz_in_ground[3] = foot_position_start_point[3] + Ellipse->get_x();
		temp_xyz_in_ground[4] = foot_position_start_point[4] + Ellipse->get_y();
		temp_xyz_in_ground[5] = foot_position_start_point[5] + Ellipse->get_z();
		//�滮leg4
		temp_xyz_in_ground[9] = foot_position_start_point[9] + Ellipse->get_x();
		temp_xyz_in_ground[10] = foot_position_start_point[10] + Ellipse->get_y();
		temp_xyz_in_ground[11] = foot_position_start_point[11] + Ellipse->get_z();
		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 1 * 3, current_leg + 1 * 3);
		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 3 * 3, current_leg + 3 * 3);
	}


	//ÿ���һ���������ߺ��¼һ�νŵ�λ��
	if (count + 1 == floor(Ellipse->get_s().getTc() * 1000))
	{
		for (int i = 0; i < 12; ++i)
		{
			foot_position_start_point[i] = current_leg[i];
		}
	}
}

//------------------------------------------------�滮����----------------------------------------------//

//���������ڹ滮����������ڶԽǲ�̬�������λ�ù켣��������ת����̬�任��
//��ǰ�����λ�� = ��һ�������λ�� + ����λ������
//ÿ������һ�������ǣ������λ�ø���һ��
//#ע�⣺Ŀǰֻ������ƽ������
auto planBodyTransformTrot(int e_1, int n, double* current_body, int count, EllipseTrajectory* Ellipse)->void
{
	int per_step_count = Ellipse->get_s().getTc() * 1000;
	if (count == 0) //���ã�����ɾ�������㲻���Ƕ�
	{
		for (int i = 0; i < 16; ++i)
		{
			current_body[i] = body_position_start_point[i];
		}
	}

	if (e_1 == 0)   //���ٶ�
	{
		//�滮����
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() * count * count / (4.0 * per_step_count * per_step_count);
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() * count * count / (4.0 * per_step_count * per_step_count);
	}
	else if (e_1 == (2 * n - 1))//���ٶ�
	{

		//�滮����
		int t = (2 * n - 1) * per_step_count + per_step_count;
		current_body[3] = body_position_start_point[3] + 0 - Ellipse->get_a() * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + Ellipse->get_a() * n - Ellipse->get_a() / 2.0;//n * a
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + 0 - Ellipse->get_c() * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + Ellipse->get_c() * n - Ellipse->get_c() / 2.0;
	}
	else //���ٶ�
	{
		//�滮����
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() / 4.0 + Ellipse->get_a() * (count - per_step_count) / per_step_count / 2;//�ٶ�Ϊ100mm/s  ÿ�����per_step_count
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() / 4.0 + Ellipse->get_c() * (count - per_step_count) / per_step_count / 2;
	}

	if (count + 1 >= 2 * n * per_step_count)
	{
		for (int i = 0; i < 16; ++i)
		{
			body_position_start_point[i] = current_body[i];
		}
	}
}

//���������ڹ滮�����������walk��̬�������λ�ù켣��������ת����̬�任��
//��ǰ�����λ�� = ��һ�������λ�� + ����λ������
//ÿ������һ�������ǣ������λ�ø���һ��
//#ע�⣺Ŀǰֻ������ƽ������
auto planBodyTransformWalk(int e_1, int n, double* current_body, int count, EllipseTrajectory* Ellipse)->void
{
	int per_step_count = Ellipse->get_s().getTc() * 1000;
	if (count == 0) //���ã�����ɾ�������㲻���Ƕ�
	{
		for (int i = 0; i < 16; ++i)
		{
			current_body[i] = body_position_start_point[i];
		}
	}

	if (e_1 == 0 || e_1 == 1)   //���ٶ�
	{
		//�滮����
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() * count * count / (4.0 * 2 * per_step_count * 2 * per_step_count);
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() * count * count / (4.0 * 2 * per_step_count * 2 * per_step_count);
	}
	else if (e_1 == (4 * n - 1) || e_1 == (4 * n - 2))//���ٶ�
	{

		//�滮����
		int t = 4 * n * per_step_count;
		current_body[3] = body_position_start_point[3] + 0 - Ellipse->get_a() * (count - t) * (count - t) / (4.0 * 2*per_step_count * 2*per_step_count) + Ellipse->get_a() * n - Ellipse->get_a() / 2.0;//n * a 
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + 0 - Ellipse->get_c() * (count - t) * (count - t) / (4.0 * 2*per_step_count * 2*per_step_count) + Ellipse->get_c() * n - Ellipse->get_c() / 2.0;
	}
	else //���ٶ�
	{
		//�滮���壬�ڼ��ٶεĻ����ϼ��㡣б�ʿ��Զ�
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() / 4.0 + Ellipse->get_a() * (count - 2 * per_step_count) / per_step_count / 4.0;//�ٶ�Ϊ100mm/s  ÿ�����per_step_count
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() / 4.0 + Ellipse->get_c() * (count - 2 * per_step_count) / per_step_count / 4.0;
	}

	if (count + 1 >= 4 * n * per_step_count)
	{
		for (int i = 0; i < 16; ++i)
		{
			body_position_start_point[i] = current_body[i];
		}
	}
}


//���������ڹ滮����������������̬�任
//#ע�⣺ֻ�����ڵ��Ų���ʱ������Ť��������
//���ܻ���ֵ�����   body_pisiton_start_point*R_z
auto planBodyRotation(int count, double* current_body, BodyPose* body_pose_param)->void
{
	static double pitch = 0;
	static double roll = 0;
	static double yaw = 0;
	//��ʼʱ��ȡ֮ǰ��ֵ
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
		aris::dynamic::s_pm_dot_pm(R_z, body_position_start_point, tempz);
		std::copy(tempz, tempz + 16, current_body);
	}
	else if (body_pose_param->getPitchTotalAngle() == 0 && body_pose_param->getRollTotalAngle() != 0 && body_pose_param->getYawTotalAngle() == 0) //roll
	{
		aris::dynamic::s_pm_dot_pm(body_position_start_point, R_x, tempx);
		std::copy(tempx, tempx + 16, current_body);
	}
	else //yaw
	{
		aris::dynamic::s_pm_dot_pm(body_position_start_point, R_y, tempy);
		std::copy(tempy, tempy + 16, current_body);
	}
	//����ʱ����仯֮���ֵ
	if (count + 1 == floor(body_pose_param->getTcurve().getTc() * 1000))
	{
		body_pose_start_pitch = pitch;
		body_pose_start_roll = roll;
		body_pose_start_yaw = yaw;
	}
}


//���������ڹ滮����������¶׺�����  
//#ע�⣺ֻ�����ڵ��Ų���ʱ���������¶������
auto planBodyUpDown(int count, double* current_body, EllipseTrajectory* body_p_param,double distance)->void
{

	//��ʼʱ��ȡ֮ǰ��ֵ
	if (count == 0) //���ã�����ɾ�������㲻���Ƕ�
	{
		for (int i = 0; i < 16; ++i)
		{
			current_body[i] = body_position_start_point[i];
		}
	}

	current_body[7] = body_position_start_point[7] + distance * body_p_param->get_s().getTCurve(count);

	//����ʱ����仯֮���ֵ
	if (count + 1 == floor(body_p_param->get_s().getTc() * 1000))
	{
		for (int i = 0; i < 16; ++i)
		{
			body_position_start_point[i] = current_body[i];
		}
	}
}

//���������ڹ滮���������ԭ����ת
//ÿһ����������ת�������Ƕȵ�һ�룬��trot��̬��Ӧ
//count �� 0 -> count
auto planBodyTurn(int count, double* current_body, BodyPose* body_pose_param)->void
{
	double yaw = 0;

	//ÿ���������߿�ʼʱ��ȡ֮ǰ��ֵ
	if (count == 0)
	{
		//yaw = body_pose_start_yaw;
	}
	if (count == 0) //���ã�����ɾ�������㲻���Ƕ�
	{
		for (int i = 0; i < 16; ++i)
		{
			current_body[i] = body_position_start_point[i];
		}
	}
	body_pose_param->getBodyRotationTrajectory(count);

	yaw = body_pose_param->getCurrentYaw() / 2;

	double R_y[16] = {
						 std::cos(yaw), 0, std::sin(yaw), 0,
								0, 1,        0, 0,
						-std::sin(yaw), 0, std::cos(yaw), 0,
								0, 0,        0, 1
	};

	double tempy[16] = { 0 };

	aris::dynamic::s_pm_dot_pm(body_position_start_point, R_y, tempy);
	std::copy(tempy, tempy + 16, current_body);

	//����ʱ����仯֮���ֵ
	if (count + 1 == floor(body_pose_param->getTcurve().getTc() * 1000))
	{

		for (int i = 0; i < 16; i++)
		{
			body_position_start_point[i] = current_body[i];
		}
	}
}

//--------------------------------------------------��̬�滮---------------------------------------------------------//

//���º�����robot.cpp�б�����
//������ԭ��Ť����̬������ԭ�ظ����������ƫ��
//#ע�⣺ֻ����ɵ�����һ�����Ҫʵ���ȸ�����ƫ��������ȸ���������ָ�����ʼλ�ò��ܽ���ƫ��
auto posePlan(int count, EllipseTrajectory* Ellipse, BodyPose* body_pose_param, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;


	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };

	//�ж�����״̬
	int e_1 = count / per_step_count;  //�жϵ�ǰ������һ��,����һ��,e1��1
	int e_2 = count % per_step_count;  //0->Tc count

	//�滮��
	planLegTrot(0, 1, current_leg_in_ground, e_2, Ellipse);
	//�滮������̬
	planBodyRotation(count, current_body_in_ground, body_pose_param);
	//ģ�Ͳ���ʹ��
	for (int i = 0; i < 12; ++i)
	{
		file_current_leg[i] = current_leg_in_ground[i];
	}
	for (int i = 0; i < 16; ++i)
	{
		file_current_body[i] = current_body_in_ground[i];
	}
	//ģ�Ͳ���ʹ��

	inverseSame(current_leg_in_ground, current_body_in_ground, input);

	return  per_step_count - count - 1;
}


//-------------------�����Ⱥ���-------------------//

//���������߶Խǲ�̬������ԭ��̤����ǰ�������ˡ����ơ����ơ�
//���в������ߺͲ��������û����롣��һ����ʱ�䣨�����߿����������û������������ߵ��ٶȺͼ��ٶ�ȷ��
//#ע�⣺��������ٶȺͼ��ٶȻ�û����
auto trotPlanSameLeg(int n, int count, EllipseTrajectory* Ellipse, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };
	//�ж�����״̬
	int e_1 = count / per_step_count;  //�жϵ�ǰ������һ��,����һ��,e1��1
	int e_2 = count % per_step_count;  //0->Tc count


	//�滮��
	planLegTrot(e_1, n, current_leg_in_ground, e_2, Ellipse);
	//�滮����
	planBodyTransformTrot(e_1, n, current_body_in_ground, count, Ellipse);

	//ģ�Ͳ���ʹ��
	for (int i = 0; i < 12; ++i)
	{

		file_current_leg[i] = current_leg_in_ground[i];
	}
	for (int i = 0; i < 16; ++i)
	{
		file_current_body[i] = current_body_in_ground[i];
	}
	//ģ�Ͳ���ʹ��
	inverseSame(current_leg_in_ground, current_body_in_ground, input);

	return 2 * n * per_step_count - count - 1;
}


//���������߾�̬��̬������ԭ��̤����ǰ�������ˡ����ơ����ơ������������ȳ�ʼ��̬һ�������
//���в������ߺͲ��������û����롣��һ����ʱ�䣨�����߿����������û������������ߵ��ٶȺͼ��ٶ�ȷ��
//#ע�⣺��������ٶȺͼ��ٶȻ�û����
auto walkPlanSameLeg(int n, int count, EllipseTrajectory* Ellipse, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };

	//�ж�����״̬
	int e_1 = count / per_step_count;  //�жϵ�ǰ������һ��,����һ��,e1��1
	int e_2 = count % per_step_count;  //0->Tc count

	//�滮��
	planLegWalk(e_1, n, current_leg_in_ground, e_2, Ellipse);
	//�滮����
	planBodyTransformWalk(e_1, n, current_body_in_ground, count, Ellipse);
	//ģ�Ͳ���ʹ��
	for (int i = 0; i < 12; ++i)
	{
		file_current_leg[i] = current_leg_in_ground[i];
	}
	for (int i = 0; i < 16; ++i)
	{
		file_current_body[i] = current_body_in_ground[i];
	}
	//ģ�Ͳ���ʹ��
	inverseSame(current_leg_in_ground, current_body_in_ground, input);

	return 4 * n * per_step_count - count - 1;
}

auto updownPlanSameLeg(int count, EllipseTrajectory* Ellipse, double distance, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };

	//�ж�����״̬
	int e_1 = count / per_step_count;  //�жϵ�ǰ������һ��,����һ��,e1��1
	int e_2 = count % per_step_count;  //0->Tc count

	//�滮��
	planLegTrot(0, 1, current_leg_in_ground, e_2, Ellipse);
	//�滮����
	planBodyUpDown(count, current_body_in_ground, Ellipse, distance);

	//ģ�Ͳ���ʹ��
	for (int i = 0; i < 12; ++i)
	{
		file_current_leg[i] = current_leg_in_ground[i];
	}
	for (int i = 0; i < 16; ++i)
	{
		file_current_body[i] = current_body_in_ground[i];
	}
	//ģ�Ͳ���ʹ��
	inverseSame(current_leg_in_ground, current_body_in_ground, input);

	return  per_step_count - count - 1;
}

//�Խǲ�̬��ԭ����ת
auto turnPlanTrotSameLeg(int n, int count, EllipseTrajectory* Ellipse, BodyPose* body_pose_param, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };

	//�ж�����״̬
	int e_1 = count / per_step_count;  //�жϵ�ǰ������һ��,����һ��,e1��1
	int e_2 = count % per_step_count;  //0->Tc count

	//�滮��
	planLegTrotTurn(e_1, current_leg_in_ground, e_2, Ellipse, body_pose_param);
	//�滮����
	planBodyTurn(e_2, current_body_in_ground, body_pose_param);


	//ģ�Ͳ���ʹ��
	for (int i = 0; i < 12; ++i)
	{
		file_current_leg[i] = current_leg_in_ground[i];
	}
	for (int i = 0; i < 16; ++i)
	{
		file_current_body[i] = current_body_in_ground[i];
	}
	//ģ�Ͳ���ʹ��

	inverseSame(current_leg_in_ground, current_body_in_ground, input);

	return  per_step_count * n * 2 - count - 1;
}



//-------------------������������-------------------//

//���������߶Խǲ�̬������ԭ��̤����ǰ�������ˡ����ơ����ơ�
//���в������ߺͲ��������û����롣��һ����ʱ�䣨�����߿����������û������������ߵ��ٶȺͼ��ٶ�ȷ��
//#ע�⣺��������ٶȺͼ��ٶȻ�û����
auto trotPlanSymmetryLeg(int n, int count, EllipseTrajectory* Ellipse, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };


	//�ж�����״̬
	int e_1 = count / per_step_count;  //�жϵ�ǰ������һ��,����һ��,e1��1
	int e_2 = count % per_step_count;  //0->Tc count

	//�滮��
	planLegTrot(e_1, n, current_leg_in_ground, e_2, Ellipse);
	//�滮����
	planBodyTransformTrot(e_1, n, current_body_in_ground, count, Ellipse);

	//ģ�Ͳ���ʹ��
	for (int i = 0; i < 12; ++i)
	{
		file_current_leg[i] = current_leg_in_ground[i];
	}
	for (int i = 0; i < 16; ++i)
	{
		file_current_body[i] = current_body_in_ground[i];
	}
	//ģ�Ͳ���ʹ��
	inverseSymmetry(current_leg_in_ground, current_body_in_ground, input);

	return 2 * n * per_step_count - count - 1;
}
//���������߾�̬��̬������ԭ��̤����ǰ�������ˡ����ơ����ơ�������ǰ���ȶԳƵ����
//���в������ߺͲ��������û����롣��һ����ʱ�䣨�����߿����������û������������ߵ��ٶȺͼ��ٶ�ȷ��
//#ע�⣺��������ٶȺͼ��ٶȻ�û����
auto walkPlanSymmetryLeg(int n, int count, EllipseTrajectory* Ellipse, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };

	//�ж�����״̬
	int e_1 = count / per_step_count;  //�жϵ�ǰ������һ��,����һ��,e1��1
	int e_2 = count % per_step_count;  //0->Tc count
	//�滮��
	planLegWalk(e_1, n, current_leg_in_ground, e_2, Ellipse);
	//�滮����
	planBodyTransformWalk(e_1, n, current_body_in_ground, count, Ellipse);
	//ģ�Ͳ���ʹ��
	for (int i = 0; i < 12; ++i)
	{
		file_current_leg[i] = current_leg_in_ground[i];
	}
	for (int i = 0; i < 16; ++i)
	{
		file_current_body[i] = current_body_in_ground[i];
	}
	//ģ�Ͳ���ʹ��
	inverseSymmetry(current_leg_in_ground, current_body_in_ground, input);

	return 4 * n * per_step_count - count - 1;
}

auto updownPlanSymmetry(int count, EllipseTrajectory* Ellipse, double distance, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[12] = { 0 };
	static double current_body_in_ground[16] = { 0 };

	//�ж�����״̬
	int e_1 = count / per_step_count;  //�жϵ�ǰ������һ��,����һ��,e1��1
	int e_2 = count % per_step_count;  //0->Tc count

	//�滮��
	planLegTrot(0, 1, current_leg_in_ground, e_2, Ellipse);
	//�滮����
	planBodyUpDown(count, current_body_in_ground, Ellipse, distance);

	//ģ�Ͳ���ʹ��
	for (int i = 0; i < 12; ++i)
	{
		file_current_leg[i] = current_leg_in_ground[i];
	}
	for (int i = 0; i < 16; ++i)
	{
		file_current_body[i] = current_body_in_ground[i];
	}
	//ģ�Ͳ���ʹ��
	inverseSymmetry(current_leg_in_ground, current_body_in_ground, input);


	return  per_step_count - count - 1;
}
