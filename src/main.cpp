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
//	cs.resetModel(robot::createModelQuadruped().release());


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
