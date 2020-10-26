#include <aris.hpp>
#include "robot.h"



int main(int argc, char *argv[])
{
	auto&cs = aris::server::ControlServer::instance();
	
    cs.resetController(robot::createControllerQuadruped().release());
    cs.resetPlanRoot(robot::createPlanQuadruped().release());

    cs.init();

	//开启WebSocket/socket服务器//
    cs.open(); 

	aris::dynamic::Model m;

	// 添加杆件 //
	double pe[6]{ 0,1,0 };
	auto& p1 = m.addPartByPe(std::array<double, 6>{0,0,0,0,0,0}.data(), "321");
	auto& p2 = m.addPartByPe(std::array<double, 6>{0,0,0,0,0,0}.data(), "321");


	// 添加关节 //
	auto& j1 = m.addRevoluteJoint(p1, m.ground(), std::array<double, 3>{0, 1, 0}.data(), std::array<double, 3>{0, 0, 1}.data());
	auto& j2 = m.addRevoluteJoint(p2, p1, std::array<double, 3>{0, 2, 0}.data(), std::array<double, 3>{0, 0, 1}.data());

	// 添加驱动 //
	auto& m1 = m.addMotion(j1);
	auto& m2 = m.addMotion(j2);

	// 添加末端 //
	double ee_i_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	auto& makI = p2.addMarker("tool0", ee_i_pm);
	auto& makJ = m.ground().addMarker("wobj0", ee_j_pm);
	auto& ee = m.generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

	// 添加求解器 //
	auto& inverse_kinematic = m.solverPool().add<aris::dynamic::InverseKinematicSolver>();
	auto& forward_kinematic = m.solverPool().add<aris::dynamic::ForwardKinematicSolver>();
	auto& inverse_dynamic = m.solverPool().add<aris::dynamic::InverseDynamicSolver>();
	auto& forward_dynamic = m.solverPool().add<aris::dynamic::ForwardDynamicSolver>();


	// 添加仿真器 //
	//auto& adams =  m.simulatorPool().add<aris::dynamic::AdamsSimulator>();


	m.init();






	// position //
	m1.setMp(0.2);
	m2.setMp(0.5);

	std::cout << forward_kinematic.kinPos() << std::endl;

	ee.updMpm();

	double pe2[6];
	ee.getMpe(pe2,"321");

	aris::dynamic::dsp(1, 6, pe2);


	// jocobian //
	forward_kinematic.cptJacobi();
	aris::dynamic::dsp(forward_kinematic.mJf(), forward_kinematic.nJf(), forward_kinematic.Jf());
	forward_kinematic.cptJacobiWrtEE();// 末端雅克比

	// velocity //
	m1.setMv(0.3);
	m2.setMv(0.8);


	forward_kinematic.kinVel();
	ee.getMvs(pe2);

	aris::dynamic::dsp(1, 6, pe2);


	// acc //
	m1.setMa(0.5);
	m2.setMa(0.8);
	inverse_dynamic.dynAccAndFce();

	

	ee.updMas();
	ee.getMas(pe2);

	aris::dynamic::dsp(1, 6, pe2);


	std::cout << m1.mf() << std::endl;
	std::cout << m2.mf() << std::endl;


	inverse_dynamic.cptGeneralInverseDynamicMatrix();
	aris::dynamic::dsp(inverse_dynamic.nM(), inverse_dynamic.nM(), inverse_dynamic.M());
	aris::dynamic::dsp(inverse_dynamic.nM(), 1, inverse_dynamic.h());





	//adams.saveAdams("C:\\Users\\DELL1\\Desktop\\aaa");




	//等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
	cs.runCmdLine();

	return 0;
}
