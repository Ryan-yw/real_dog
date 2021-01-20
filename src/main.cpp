#include <aris.hpp>
#include "robot.h"
#include"kinematics.h"
#include"plan.h"


extern double input_angle[12];
extern double file_current_leg[12];
extern double file_current_body[16];
class Quad :public aris::dynamic::Model {
public:

	Quad() {
		// add part //
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto& body = this->partPool().add<aris::dynamic::Part>("BODY", default_iv);
		//leg1
		auto& lf_p1 = this->partPool().add<aris::dynamic::Part>("LF_P1", default_iv);
		auto& lf_p2 = this->partPool().add<aris::dynamic::Part>("LF_P2", default_iv);
		auto& lf_p3 = this->partPool().add<aris::dynamic::Part>("LF_P3", default_iv);
		//leg2
		auto& lr_p1 = this->partPool().add<aris::dynamic::Part>("LR_P1", default_iv);
		auto& lr_p2 = this->partPool().add<aris::dynamic::Part>("LR_P2", default_iv);
		auto& lr_p3 = this->partPool().add<aris::dynamic::Part>("LR_P3", default_iv);
		//leg3
		auto& rr_p1 = this->partPool().add<aris::dynamic::Part>("RR_P1", default_iv);
		auto& rr_p2 = this->partPool().add<aris::dynamic::Part>("RR_P2", default_iv);
		auto& rr_p3 = this->partPool().add<aris::dynamic::Part>("RR_P3", default_iv);
		//leg4
		auto& rf_p1 = this->partPool().add<aris::dynamic::Part>("RF_P1", default_iv);
		auto& rf_p2 = this->partPool().add<aris::dynamic::Part>("RF_P2", default_iv);
		auto& rf_p3 = this->partPool().add<aris::dynamic::Part>("RF_P3", default_iv);
		// add geometry //


		body.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\body.x_t");
		//leg1
		lf_p1.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l11.x_t");
		lf_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l12.x_t");
		lf_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l13.x_t");
		//leg2
		lr_p1.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l21.x_t");
		lr_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l22.x_t");
		lr_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l23.x_t");
		//leg3
		rr_p1.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l31.x_t");
		rr_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l32.x_t");
		rr_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l33.x_t");
		//leg4
		rf_p1.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l41.x_t");
		rf_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l42.x_t");
		rf_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l43.x_t");

		//定义关节的位置
		const double leg_pe[12][3]{
			{  kBodyLong / 2,   0, -kBodyWidth / 2      },
			{  kBodyLong / 2,   0, -kBodyWidth / 2 - L1 },
			{  kBodyLong / 2, -L2, -kBodyWidth / 2 - L1 },
			{ -kBodyLong / 2,   0, -kBodyWidth / 2      },
			{ -kBodyLong / 2,   0, -kBodyWidth / 2 - L1 },
			{ -kBodyLong / 2, -L2, -kBodyWidth / 2 - L1 },
			{ -kBodyLong / 2,   0,  kBodyWidth / 2      },
			{ -kBodyLong / 2,   0,  kBodyWidth / 2 + L1 },
			{ -kBodyLong / 2, -L2,  kBodyWidth / 2 + L1 },
			{  kBodyLong / 2,   0,  kBodyWidth / 2      },
			{  kBodyLong / 2,   0,  kBodyWidth / 2 + L1 },
			{  kBodyLong / 2, -L2,  kBodyWidth / 2 + L1 },
		};
		//定义末端位置
		const double ee_pos[4][6]{
		{ kBodyLong / 2, -L3 - L2, -(kBodyWidth / 2) - L1,0.0, 0.0, 0.0},   //leg1 ->012
		{-kBodyLong / 2, -L3 - L2, -(kBodyWidth / 2) - L1,0.0, 0.0, 0.0},   //leg2 ->345
		{-kBodyLong / 2, -L3 - L2,  (kBodyWidth / 2) + L1,0.0, 0.0, 0.0},   //leg3 ->678
		{ kBodyLong / 2, -L3 - L2,  (kBodyWidth / 2) + L1,0.0, 0.0, 0.0},   //leg4 ->91011
		};

		// add joints //
		//leg1
		auto& lf_r1 = this->addRevoluteJoint(lf_p1, body, leg_pe[0], std::array<double, 3>{-1, 0, 0}.data());
		auto& lf_r2 = this->addRevoluteJoint(lf_p2, lf_p1, leg_pe[1], std::array<double, 3>{0, 0, -1}.data());
		auto& lf_r3 = this->addRevoluteJoint(lf_p3, lf_p2, leg_pe[2], std::array<double, 3>{0, 0, -1}.data());
		//leg2
		auto& lr_r1 = this->addRevoluteJoint(lr_p1, body, leg_pe[3], std::array<double, 3>{-1, 0, 0}.data());
		auto& lr_r2 = this->addRevoluteJoint(lr_p2, lr_p1, leg_pe[4], std::array<double, 3>{0, 0, -1}.data());
		auto& lr_r3 = this->addRevoluteJoint(lr_p3, lr_p2, leg_pe[5], std::array<double, 3>{0, 0, -1}.data());
		//leg3
		auto& rr_r1 = this->addRevoluteJoint(rr_p1, body, leg_pe[6], std::array<double, 3>{1, 0, 0}.data());
		auto& rr_r2 = this->addRevoluteJoint(rr_p2, rr_p1, leg_pe[7], std::array<double, 3>{0, 0, 1}.data());
		auto& rr_r3 = this->addRevoluteJoint(rr_p3, rr_p2, leg_pe[8], std::array<double, 3>{0, 0, 1}.data());
		//leg4
		auto& rf_r1 = this->addRevoluteJoint(rf_p1, body, leg_pe[9], std::array<double, 3>{1, 0, 0}.data());
		auto& rf_r2 = this->addRevoluteJoint(rf_p2, rf_p1, leg_pe[10], std::array<double, 3>{0, 0, 1}.data());
		auto& rf_r3 = this->addRevoluteJoint(rf_p3, rf_p2, leg_pe[11], std::array<double, 3>{0, 0, 1}.data());
		
		// add motion //
		//leg1
		auto& lf_m1 = this->addMotion(lf_r1);
		auto& lf_m2 = this->addMotion(lf_r2);
		auto& lf_m3 = this->addMotion(lf_r3);
		//leg2
		auto& lr_m1 = this->addMotion(lr_r1);
		auto& lr_m2 = this->addMotion(lr_r2);
		auto& lr_m3 = this->addMotion(lr_r3);
		//leg3
		auto& rr_m1 = this->addMotion(rr_r1);
		auto& rr_m2 = this->addMotion(rr_r2);
		auto& rr_m3 = this->addMotion(rr_r3);
		//leg4
		auto& rf_m1 = this->addMotion(rf_r1);
		auto& rf_m2 = this->addMotion(rf_r2);
		auto& rf_m3 = this->addMotion(rf_r3);


		// add end-effector //
		auto body_ee_maki = body.addMarker("body_ee_mak_i");
		auto body_ee_makj = ground().addMarker("body_ee_mak_j");

		auto& body_ee = this->generalMotionPool().add<aris::dynamic::GeneralMotion>("body_ee", &body_ee_maki, &body_ee_makj);
		auto& lf_ee = this->addPointMotion(lf_p3, ground(), ee_pos[0]);
		ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
		auto& lr_ee = this->addPointMotion(lr_p3, ground(), ee_pos[1]);
		ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
		auto& rr_ee = this->addPointMotion(rr_p3, ground(), ee_pos[2]);
		ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
		auto& rf_ee = this->addPointMotion(rf_p3, ground(), ee_pos[3]);
		ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());

		body_ee.activate(true);

		lf_ee.activate(false);
		lr_ee.activate(false);
		rr_ee.activate(false);
		rf_ee.activate(false);

		auto& inverse_kinematic_solver = this->solverPool().add<aris::dynamic::InverseKinematicSolver>();
		auto& forward_kinematic_solver = this->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
		auto& inverse_dynamic_solver = this->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto& forward_dynamic_solver = this->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		auto& adams = this->simulatorPool().add<aris::dynamic::AdamsSimulator>();
		this->init();
	};
};


class DogDynamicTest :public aris::core::CloneObject<DogDynamicTest, aris::plan::Plan> {
public:
	auto virtual executeRT()->int override 
	{
		int ret=0;
		TCurve s1(1, 6);
		s1.getCurveParam();
		EllipseTrajectory e1( 0,0.080, 0, s1);
		BodyPose body_s(0,20,0,s1);
		// 前后左右 //
		//ret = trotPlanSameLeg(10, count() - 1, &e1, input_angle);
		
		// 原地旋转 //
		ret = turnPlanTrotSameLeg(5,count()-1, &e1, &body_s, input_angle);


		// 身子扭动 //
		//ret = posePlan(count() - 1, &e1, &body_s, input_angle);
		static double ee0[28]; 
		double ee[28];
		if (count() == 1) {
			model()->getOutputPos(ee0);
			aris::dynamic::s_vc(28, ee0, ee);
		}
	
		aris::dynamic::s_vc(16, file_current_body+0, ee+0);
		aris::dynamic::s_vc(12, file_current_leg+0, ee + 16); 

		//aris::dynamic::dsp(4, 4, ee);
		//aris::dynamic::dsp(4, 3, ee + 16);
		model()->setOutputPos(ee);
		if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
		model()->setTime(0.001 * count());
		return ret;


	}
	explicit DogDynamicTest()
	{

	}
};


int main(int argc, char *argv[])
{
	// 设置模型初始位置，给关节角度  注：相对的位置是模型Quad里设置的关节轴和末端 //
	Quad quad;
	double set_init_position[12] = { 
		0,0.736615,-1.38589,
		0,0.729938,-1.37362,
		0,-0.736615,1.38589,
		0,-0.729938,1.37362, };
	quad.setInputPos(set_init_position);
	if (quad.forwardKinematics()) THROW_FILE_LINE("forward failed");

	// 添加仿真器和仿真结果 //
	auto& adams = dynamic_cast<aris::dynamic::AdamsSimulator&>(quad.simulatorPool().front());
	auto& result = quad.simResultPool().add<aris::dynamic::SimResult>();
	quad.init();

	//adams.saveAdams("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\quad.cmd");

	// 添加规划曲线 //
	//robot::DogForward plan;
	


	DogDynamicTest plan;
	//plan.prepareNrt();
	
	adams.simulate(plan, result);
	adams.saveAdams("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\quad_simulation.cmd", result);
	
	std::cout << "simulate finished" << std::endl;

//-------------------------------------------------分割线---------------------------------------------------------//
	auto&cs = aris::server::ControlServer::instance();
	
    cs.resetController(robot::createControllerQuadruped().release());
    cs.resetPlanRoot(robot::createPlanQuadruped().release());
	cs.resetModel(new Quad);

	//网页控制代码
	//cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::WEB);
	//cs.interfacePool().add<aris::server::HttpInterface>("", "8001", "D:/UI/www");

    cs.init();
 
	//开启WebSocket/socket服务器//
    cs.open(); 


	//等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
	cs.runCmdLine();

	return 0;
}
