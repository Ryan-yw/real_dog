
#include <iostream>
#include <thread>

#include <QApplication>
#include <QDebug>

#include <aris.hpp>
#include <./control/include/imu.h>
#include "./control/include/spline.h"
#include "./control/include/mainwindow.h"
#include "./control/include/robot.h"
#include "./control/include/kinematics.h"
#include "./control/include/plan.h"


extern double input_angle[12];
extern double file_current_leg[12];
extern double file_current_body[16];

extern std::atomic<double> imu_angle[3];

int main(int argc, char *argv[])
{



    // imu thread //
       auto imu_thread = std::thread([&]()
       {
           bool first_start = true;

           try
           {
               auto ret = imu::initIMU();
               imu::startIMU();
               while(true)
               {
                   auto &data = imu::getCurrentData();
                   if(first_start)
                   {
                       std::cout << "AAAAAAAA" << std::endl;
                       data.yaw_offset_ = data.yaw;
                       first_start = false;
                       std::cout << first_start << "\t" << data.yaw_offset_ << std::endl;
                   }

                   imu_angle[0] = data.pitch;
                   imu_angle[1] = data.roll;
                   imu_angle[2] = data.yaw - data.yaw_offset_;
               }
           }
           catch(std::exception &e)
           {
               std::cout<<"imu not connected: "<<e.what()<<std::endl;
           }
       });

       // plot thread //
       auto airs_th = std::thread([&]()
       {

           QApplication a(argc, argv);
           MainWindow w;
           w.show();
           return a.exec();

       });

      // while(true);

	auto&cs = aris::server::ControlServer::instance();


    cs.resetController(robot::createControllerQuadruped().release());
    cs.resetPlanRoot(robot::createPlanQuadruped().release());
    cs.resetModel(robot::createModelQuadruped().release());
	


	// 设置模型初始位置，给关节角度  注：相对的位置是模型Quad里设置的关节轴和末端 //

//    double set_init_position[12] = {
//    0,-0.538643,1.0696,
//    0,-0.538637,1.06958,
//    0,-0.538643,1.0696,
//    0,-0.538637,1.06958
//    };
//    //	double set_init_position[12] = {
//    //0,0.5486235746,-1.0795543012,
//    //0,0.5386215555,-1.0695502982,
//    //0,-0.5386235746,1.0695543012,
//    //0,-0.5386215555,1.0695502982
//    //	};

//    cs.model().setInputPos(set_init_position);
//    if (cs.model().forwardKinematics()) THROW_FILE_LINE("forward failed");

#ifdef WIN32

	//---------------------------------adams仿真------------------------------------------------//

	// 运动学和动力学adams测试 //
	auto& adams = dynamic_cast<aris::dynamic::AdamsSimulator&>(cs.model().simulatorPool().front());
	//adams.saveAdams("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\quad.cmd");

	robot::DogDynamicTest plan;
	adams.simulate(plan, cs.model().simResultPool().front());
	adams.saveAdams("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\quad_simulation.cmd", cs.model().simResultPool().front());

	//adams.saveAdams("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\quad_simulation.cmd", cs.model().simResultPool().front(), 1200);

	std::cout << "simulate finished" << std::endl;
	//---------------------------------adams仿真------------------------------------------------//

#endif // WIN32



	




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
