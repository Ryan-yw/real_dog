#include <aris.hpp>
#include "robot.h"
#include"kinematics.h"
#include"plan.h"


extern double input_angle[12];
extern double file_current_leg[12];
extern double file_current_body[16];

int main(int argc, char *argv[])
{
	
	auto&cs = aris::server::ControlServer::instance();
	
    cs.resetController(robot::createControllerQuadruped().release());
    cs.resetPlanRoot(robot::createPlanQuadruped().release());
	cs.resetModel(robot::createModelQuadruped().release());
	

	// 设置模型初始位置，给关节角度  注：相对的位置是模型Quad里设置的关节轴和末端 //

	double set_init_position[12] = {
0,0.7299607015,-1.373657219,
0,0.7299380908,-1.3736156494,
0,-0.7299607015,1.373657219,
0,-0.7299380908,1.3736156494 };
//	double set_init_position[12] = {
//
//	0.133743,
//0.771029,
//- 1.44897,
//0.133055,
//0.76439,
//- 1.43682,
//0.133743,
//- 0.771029,
//1.44897,
//0.133055,
//- 0.76439,
//1.43682 };

	//cs.model().setInputPos(set_init_position);
	//if (cs.model().forwardKinematics()) THROW_FILE_LINE("forward failed");

	//robot::setStandTopologyIK(cs);
	////robot::setBoundTopologyIK(cs);
	//
	//auto& adams = dynamic_cast<aris::dynamic::AdamsSimulator&>(cs.model().simulatorPool().front());
	////adams.saveAdams("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\quad_simulation.cmd");
	//
	//robot::DogDynamicTest plan;
	//adams.simulate(plan, cs.model().simResultPool().front());
	//adams.saveAdams("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\quad_simulation.cmd", cs.model().simResultPool().front());

	//std::cout << "simulate finished" << std::endl;

	//网页控制代码
	cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::WEB);
	cs.interfacePool().add<aris::server::HttpInterface>("", "8001", "D:/UI/www");

    cs.init();
 
	//开启WebSocket/socket服务器//
    cs.open(); 

	//等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
	cs.runCmdLine();

	return 0;
}
