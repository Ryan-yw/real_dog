#ifndef PLAN_H_
#define PLAN_H_




///���ܣ�����0->1���������ߡ��ɸ�������ļ��ٶȺ��ٶ��ж�����Ϊ���λ���������
//   ##��������##
//  Tc:���������ܹ���Ҫ��ʱ�䣬������ļ��ٶȺ��ٶȼ���
//   v:�ٶȣ����û����룬���캯����ʼ��
//   a:���ٶȣ����û����룬���캯����ʼ��
//  ta:���ٶ������ʱ�䣬��������ٶȺͼ��ٶȼ���õ�
class TCurve
{
private:
	double Tc_;
	double v_;
	double a_;
	double ta_;

public:
	auto getTCurve(int count)->double;
	auto getCurveParam()->void;
	auto getTc()->double { return Tc_; };
	TCurve(double a, double v) { a_ = a; v_ = v; }
	~TCurve() {}
};





///���ܣ�������Բ�켣����Tcʱ����  x����0->a;y����0->b->0;z����0->c
//   ##��������##
//   a:x���򲽳������û����룬���캯����ʼ��
//   b:y���򲽸ߣ����û����룬���캯����ʼ��
//   c:z���򲽳������û����룬���캯����ʼ��
//   x:x������tʱ��ʱ��λ��
//   y:y������tʱ��ʱ��λ��
//   z:z������tʱ��ʱ��λ��
//   s:��������
class EllipseTrajectory
{
private:
	double x_;
	double y_;
	double z_;
	double a_;
	double b_;
	double c_;
	TCurve s_;

public:
	auto getEllipseTrajectory(int count)->void;
	auto get_x()->double { return x_; };
	auto get_y()->double { return y_; };
	auto get_z()->double { return z_; };
	auto get_a()->double { return a_; };
	auto get_b()->double { return b_; };
	auto get_c()->double { return c_; };
	auto get_s()->TCurve { return s_; };
	EllipseTrajectory(double a, double b, double c, TCurve& s) :a_(a), b_(b), c_(c), s_(s), x_(0), y_(0), z_(0) {}
	~EllipseTrajectory() {}
};



class Gait
{
public:
	EllipseTrajectory E;


	~Gait() {}
};



///���ܣ����ɻ�������xyz��������ת�ĽǶȹ켣
//   ##��������##
// double roll_angle_x_����x����ת�ĽǶȣ����û����룬���캯����ʼ��
// double yaw_angle_y_����y����ת�ĽǶȣ����û����룬���캯����ʼ��
// double pitch_angle_z_����z����ת�ĽǶȣ����û����룬���캯����ʼ��
// double pitch_����tʱ�̵ĸ�����
// double roll_����tʱ�̵ĺ����
// double yaw_����tʱ�̵�ƫ���ǽ�
// TCurve b_r_s_:��������
//   s:��������
class BodyPose
{
private:
	double roll_angle_x_;
	double yaw_angle_y_;
	double pitch_angle_z_;
	double pitch_;
	double roll_;
	double yaw_;
	TCurve b_r_s_;

public:
	auto getBodyRotationTrajectory(int count)->void;
	auto getCurrentRoll()->double { return roll_; };
	auto getCurrentPitch()->double { return pitch_; };
	auto getCurrentYaw()->double { return yaw_; };
	auto getRollTotalAngle()->double { return roll_angle_x_; };
	auto getPitchTotalAngle()->double { return pitch_angle_z_; };
	auto getYawTotalAngle()->double { return yaw_angle_y_; };
	auto getTcurve()->TCurve { return b_r_s_; };
	BodyPose(double roll, double yaw, double pitch, TCurve s) : roll_angle_x_(roll), yaw_angle_y_(yaw), pitch_angle_z_(pitch), b_r_s_(s), pitch_(0), yaw_(0), roll_(0) {}
	~BodyPose() {};
};






/***********************������������***********************/
auto trotPlanSameLeg(int n, int count, EllipseTrajectory* param, double* input)->int;
auto trotPlanSymmetryLeg(int n, int count, EllipseTrajectory* Ellipse, double* input)->int;
auto walkPlanSameLeg(int n, int count, EllipseTrajectory* param, double* input)->int;
auto walkPlanSymmetryLeg(int n, int count, EllipseTrajectory* Ellipse, double* input)->int;
auto posePlan(int count, EllipseTrajectory* Ellipse, BodyPose* body_pose, double* input)->int;
auto upPlan(int count, EllipseTrajectory* Ellipse, double distance, double* input)->int;
auto updownPlanSameLeg(int count, EllipseTrajectory* Ellipse, double distance, double* input)->int;
auto updownPlanSymmetry(int count, EllipseTrajectory* Ellipse, double distance, double* input)->int;
auto turnPlanTrotSameLeg(int n, int count, EllipseTrajectory* Ellipse, BodyPose* body_pose_param, double* input)->int;
#endif



