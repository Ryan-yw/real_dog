#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>

#include "./control/include/robot.h"
#include "./control/include/plan.h"
#include "./control/include/kinematics.h"
#include "./control/include/filter.h"
#include "./control/include/data.h"

double input_angle[12] = {0};      //任意时刻关节角度
double init_pos_angle[12] = { 0 }; //初始位置下的关节角度
extern std::atomic<double> gamepad[2];

std::string gait="trot";
std::string prepose="same";
//输出参数，模型曲线测试使用
double current_body_and_leg[28] = { 
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1
};


double body_cm[9]={0.1,0,-0.05,0.1,0,0.1,0,0,-0.05};

using namespace aris::dynamic;
using namespace aris::plan;

std::atomic<float> body_pos_[6];
std::atomic<float> leg_pos_[12];

extern std::atomic<double> joint_pos[12];

namespace robot
{
#ifdef UNIX


static RobotDataProcess data;

RobotDataProcess::RobotDataProcess()
{
    this->lcm_robot_estimator_.subscribe("state_estimator", &RobotDataProcess::estimaterDataProcess, this);
    this->estimator_data_robot_thread_ = std::thread([&]() { while (true) { lcm_robot_estimator_.handle(); } });
    this->lcm_robot_wbc_.subscribe("wbc_lcm_data", &RobotDataProcess::wbcDataProcess, this);
    this->wbc_data_robot_thread_ = std::thread([&]() { while (true) { lcm_robot_wbc_.handle(); } });
}
auto RobotDataProcess::estimaterDataProcess(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const state_estimator_lcmt* msg)->void
{
    body_pos_[0] = msg->p[0];
    body_pos_[1] = msg->p[1];
    body_pos_[2] = msg->p[2];
    body_pos_[3] = msg->rpy[0];
    body_pos_[4] = msg->rpy[1];
    body_pos_[5] = msg->rpy[2];
    //std::cout << body_pos_[0] << "\t" << body_pos_[1] << "\t" << body_pos_[2] << std::endl;
}
auto RobotDataProcess::wbcDataProcess(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const wbc_test_data_t* msg)->void
{
    for (int i = 0; i < 12; ++i)
    {
        leg_pos_[i] = msg->foot_pos[i];
    }

}

// muli-thread communication //
auto ControlThread::prepareNrt()->void
{
    dir_ = doubleParam("direction");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto ControlThread::executeRT()->int
{

    std::cout << "leg1:" << leg_pos_[0] << "\t" << leg_pos_[1] << "\t" << leg_pos_[2] << "\t"
        << "leg2:" << leg_pos_[3] << "\t" << leg_pos_[4] << "\t" << leg_pos_[5] << "\t"
        << "leg3:" << leg_pos_[6] << "\t" << leg_pos_[7] << "\t" << leg_pos_[8] << "\t"
        << "leg4:" << leg_pos_[9] << "\t" << leg_pos_[10] << "\t" << leg_pos_[11] << "\t";
    std::cout << "body" << body_pos_[0] << "\t" << body_pos_[1] << "\t" << body_pos_[2] << std::endl;
    return 1;

}
ControlThread::ControlThread(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"test_th\">"
        "<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
        "</Command>");
}
ControlThread::~ControlThread() = default;


#endif // UNIX




////测试滤波算法 //
//auto TestFilter::prepareNrt()->void
//{
//    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
//}
//auto TestFilter::executeRT()->int
//{
//    if (count() == 1) this->master()->logFileRawName("filter");
//    double input_value, output_value,sin;
//    sin = std::sin(count() / 1000.0);
//    input_value = std::sin(count() / 1000.0) + std::rand() * 0.25 / RAND_MAX ;
//    filter_limit_average(input_value, output_value);
//    lout() << sin << "\t" << input_value << "\t" << output_value << std::endl;
//    
//
//    return PI*1000 - count();
//}
//auto TestFilter::collectNrt()->void {}
//TestFilter::~TestFilter() = default;
//TestFilter::TestFilter(const std::string & name) {
//    aris::core::fromXmlString(command(),
//        "<Command name=\"test_filter\"/>");
//}



//设置力矩
auto SetMaxToq::prepareNrt()->void
{
    for(auto &m:motorOptions()) m = aris::plan::Plan::CHECK_NONE;
}
auto SetMaxToq::executeRT()->int
{
    auto &cout = controller()->mout();

    for (int i = 0; i < 3; ++i)
    {
        std::uint16_t max_toq=1000;
        this->ecController()->motionPool()[i].writePdo(0x6072,0,max_toq);
    }
    //返回0表示正常结束，返回负数表示报错，返回正数表示正在执行
    return 0;
}
auto SetMaxToq::collectNrt()->void {}
SetMaxToq::~SetMaxToq() = default;
SetMaxToq::SetMaxToq(const std::string &name){
    aris::core::fromXmlString(command(),
                "<Command name=\"set_toq\"/>");
}


//读力传感器
auto DogReadForce::prepareNrt()->void
{
    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogReadForce::executeRT()->int
{

    // uint N
    std::int32_t force_sensor_data[4]={0};

    this->ecController()->slavePool()[0].readPdo(0x6020,0x01,force_sensor_data[0]);
    this->ecController()->slavePool()[0].readPdo(0x6020,0x02,force_sensor_data[1]);
    this->ecController()->slavePool()[0].readPdo(0x6020,0x04,force_sensor_data[2]);
    this->ecController()->slavePool()[0].readPdo(0x6020,0x05,force_sensor_data[3]);

    for(int i=0;i<4;++i)
    {
      std::cout << force_sensor_data[i] << "\t";
    }
    std::cout << std::endl;
     return 1;
}
DogReadForce::DogReadForce(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"dog_readf\"/>");
}
DogReadForce::~DogReadForce() = default;


//读当前关节角度
auto DogReadJoint::prepareNrt()->void
{
    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogReadJoint::executeRT()->int
{
    static double begin_angle[12];
    begin_angle[0] = controller()->motionPool()[0].actualPos();
    begin_angle[1] = controller()->motionPool()[1].actualPos();
    begin_angle[2] = controller()->motionPool()[2].actualPos();
    begin_angle[3] = controller()->motionPool()[3].actualPos();
    begin_angle[4] = controller()->motionPool()[4].actualPos();
    begin_angle[5] = controller()->motionPool()[5].actualPos();
    begin_angle[6] = controller()->motionPool()[6].actualPos();
    begin_angle[7] = controller()->motionPool()[7].actualPos();
    begin_angle[8] = controller()->motionPool()[8].actualPos();
    begin_angle[9] = controller()->motionPool()[9].actualPos();
    begin_angle[10] = controller()->motionPool()[10].actualPos();
    begin_angle[11] = controller()->motionPool()[11].actualPos();


    for(int i=0;i<12;++i)
    {
       std::cout<<begin_angle[i]<<std::endl;
    }

    return 0;
}
DogReadJoint::DogReadJoint(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"dog_readj\"/>");
}
DogReadJoint::~DogReadJoint() = default;

//测试：单轴读关节扭矩
auto DogTorqueControl::prepareNrt()->void
{
    dir_ = doubleParam("direction");
    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogTorqueControl::executeRT()->int
{


    static double begin_angle[12];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motionPool()[0].actualPos();
    }

    TCurve s1(1,3);
    s1.getCurveParam();

    double angle0 = begin_angle[0] + dir_ * PI * s1.getTCurve(count());

    // read toeque //
    double torque=0;
    this->ecController()->motionPool()[0].readPdo(0x6077,0x00,torque);
    mout()<< torque <<std::endl;

    controller()->motionPool()[0].setTargetPos(angle0);
    return s1.getTc() * 1000-count();

}
DogTorqueControl::DogTorqueControl(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"dog_mvt\">"
        "<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
        "</Command>");
}
DogTorqueControl::~DogTorqueControl() = default;


//设置初始准备状态关节位置
auto DogInitPos::prepareNrt()->void
{

    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogInitPos::executeRT()->int
{
    // set home and get pos

    TCurve s1(1, 4);
    s1.getCurveParam();
    EllipseTrajectory e1(0.2, 0.05, 0, s1);


   auto ret = trotPlanSameLeg(1, count() - 1, &e1, current_body_and_leg);

    model()->setOutputPos(current_body_and_leg);
    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    for (auto m : model()->motionPool()) m.updP();
    model()->getInputPos(init_pos_angle);
    aris::dynamic::dsp(4, 3, init_pos_angle);

    return 0;
}
DogInitPos::DogInitPos(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_getpos\"/>");
}
DogInitPos::~DogInitPos() = default;


//轴空间移动
auto DogMoveJoint::prepareNrt()->void
{
    dir_ = doubleParam("direction");

    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;

}
auto DogMoveJoint::executeRT()->int
{
    static double begin_angle[12];

    if (count() == 1)
    {
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        begin_angle[1] = controller()->motionPool()[1].actualPos();
        begin_angle[2] = controller()->motionPool()[2].actualPos();
        begin_angle[3] = controller()->motionPool()[3].actualPos();
        begin_angle[4] = controller()->motionPool()[4].actualPos();
        begin_angle[5] = controller()->motionPool()[5].actualPos();
        begin_angle[6] = controller()->motionPool()[6].actualPos();
        begin_angle[7] = controller()->motionPool()[7].actualPos();
        begin_angle[8] = controller()->motionPool()[8].actualPos();
        begin_angle[9] = controller()->motionPool()[9].actualPos();
        begin_angle[10] = controller()->motionPool()[10].actualPos();
        begin_angle[11] = controller()->motionPool()[11].actualPos();
    }

        TCurve s1(5,2);
        s1.getCurveParam();

    double angle0 = begin_angle[0] + dir_ * PI * s1.getTCurve(count());
    double angle1 = begin_angle[1] + dir_ * PI * s1.getTCurve(count());
    double angle2 = begin_angle[2] + dir_ * PI * s1.getTCurve(count());
    double angle3 = begin_angle[3] + dir_ * PI * s1.getTCurve(count());
    double angle4 = begin_angle[4] + dir_ * PI * s1.getTCurve(count());
    double angle5 = begin_angle[5] + dir_ * PI * s1.getTCurve(count());
    double angle6 = begin_angle[6] + dir_ * PI * s1.getTCurve(count());
    double angle7 = begin_angle[7] + dir_ * PI * s1.getTCurve(count());
    double angle8 = begin_angle[8] + dir_ * PI * s1.getTCurve(count());
    double angle9 = begin_angle[9] + dir_ * PI * s1.getTCurve(count());
    double angle10 = begin_angle[10] + dir_ * PI * s1.getTCurve(count());
    double angle11 = begin_angle[11] + dir_ * PI * s1.getTCurve(count());



    controller()->motionPool()[0].setTargetPos(angle0);
    controller()->motionPool()[1].setTargetPos(angle1);
    controller()->motionPool()[2].setTargetPos(angle2);
    controller()->motionPool()[3].setTargetPos(angle3);
    controller()->motionPool()[4].setTargetPos(angle4);
    controller()->motionPool()[5].setTargetPos(angle5);
    controller()->motionPool()[6].setTargetPos(angle6);
    controller()->motionPool()[7].setTargetPos(angle7);
    controller()->motionPool()[8].setTargetPos(angle8);
    controller()->motionPool()[9].setTargetPos(angle9);
    controller()->motionPool()[10].setTargetPos(angle10);
    controller()->motionPool()[11].setTargetPos(angle11);
    std::cout <<s1.getTc() << std::endl;
    return s1.getTc() * 1000-count();
}
auto DogMoveJoint::collectNrt()->void {}
DogMoveJoint::DogMoveJoint(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"dog_mvj\">"
        "<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
        "</Command>");
}
DogMoveJoint::~DogMoveJoint() = default;

//从轴空间回原点
auto DogHome::prepareNrt()->void
{
    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogHome::executeRT()->int
{
    static double begin_angle[12]={0};
    double angle[12]={0};

    if (count() == 1)
    {
        this->master()->logFileRawName("home");
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        begin_angle[1] = controller()->motionPool()[1].actualPos();
        begin_angle[2] = controller()->motionPool()[2].actualPos();
//        begin_angle[3] = controller()->motionPool()[3].actualPos();
//        begin_angle[4] = controller()->motionPool()[4].actualPos();
//        begin_angle[5] = controller()->motionPool()[5].actualPos();
//        begin_angle[6] = controller()->motionPool()[6].actualPos();
//        begin_angle[7] = controller()->motionPool()[7].actualPos();
//        begin_angle[8] = controller()->motionPool()[8].actualPos();
//        begin_angle[9] = controller()->motionPool()[9].actualPos();
//        begin_angle[10] = controller()->motionPool()[10].targetPos();
//        begin_angle[11] = controller()->motionPool()[11].targetPos();
    }

    TCurve s1(5,2);//0.9s
    s1.getCurveParam();

    for(int i=0;i<3;++i)
    {
        angle[i] = begin_angle[i] + (0-begin_angle[i]) * s1.getTCurve(count());
    }

    for(int i=0;i<3;++i)
    {
        controller()->motionPool()[i].setTargetPos(angle[i]);
    }
    return s1.getTc() * 1000-count();
}
DogHome::DogHome(const std::string &name)
{
    aris::core::fromXmlString(command(),
       "<Command name=\"dog_home\"/>");
}
DogHome::~DogHome() = default;

//行走步态和（曲腿）模式设置
auto DogSetWalkMode::prepareNrt()->void
{
    gait_ = cmdParams().at("gait");
    prepose_ = cmdParams().at("prepose");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogSetWalkMode::executeRT()->int
{
    if(gait_=="trot"||gait_=="walk")
    {gait = gait_;}
    else
        mout()<<"input gait error"<<std::endl;
    if(prepose_=="same"||prepose_=="symmetry")
        prepose=prepose_;
    else
        mout()<<"input prepose error"<<std::endl;
    return 0;
}
DogSetWalkMode::DogSetWalkMode(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_setmode\">"
        "<GroupParam>"
        "<Param name=\"prepose\" default=\"same\" abbreviation=\"p\"/>"
        "<Param name=\"gait\" default=\"trot\" abbreviation=\"g\"/>"
        "</GroupParam>"
        "</Command>");
}
DogSetWalkMode::~DogSetWalkMode() = default;

//////轴空间初始位姿切换
////auto DogSwitchPrePose::prepareNrt()->void
////{
////    prepose_ = cmdParams().at("prepose");
////    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
////}
////auto DogSwitchPrePose::executeRT()->int
////{
////    static double begin_angle[12] = { 0 };
////    double angle[12] = { 0 };
////  
////    double distance_same[12] = { 0 };
////    double distance_symmetry[12] = { 0 };
////
////    if (prepose_ == "same")
////    {
////        for (int i = 0; i < 12; ++i)
////        {
////            distance_same[i] = init_pos_angle[i];
////            distance_symmetry[i] = init_pos_angle[i];
////        }
////    }
////    else if (prepose_ == "symmetry")//symmetry
////    {
////        distance_symmetry[4] = -distance_symmetry[4]; //前后腿对称，14相同，23相同
////        distance_symmetry[5] = -distance_symmetry[5];
////        distance_symmetry[7] = -distance_symmetry[7];
////        distance_symmetry[8] = -distance_symmetry[8];
////    }
////    else
////    {
////        mout() << "input error" << std::endl;
////    }
////
////    if (count() == 1)
////    {
////        this->master()->logFileRawName("switchpose");
////        begin_angle[0] = controller()->motionPool()[0].actualPos();
////        begin_angle[1] = controller()->motionPool()[1].actualPos();
////        begin_angle[2] = controller()->motionPool()[2].actualPos()/2;
////        begin_angle[3] = controller()->motionPool()[3].actualPos();
////        begin_angle[4] = controller()->motionPool()[4].actualPos();
////        begin_angle[5] = controller()->motionPool()[5].actualPos()/2;
////        begin_angle[6] = controller()->motionPool()[6].actualPos();
////        begin_angle[7] = controller()->motionPool()[7].actualPos();
////        begin_angle[8] = controller()->motionPool()[8].actualPos()/2;
////        begin_angle[9] = controller()->motionPool()[9].actualPos();
////        begin_angle[10] = controller()->motionPool()[10].actualPos();
////        begin_angle[11] = controller()->motionPool()[11].actualPos()/2;
////    }
////
////
////    TCurve s1(5, 2);//0.9s
////    s1.getCurveParam();
////
////    if (prepose_ == "same")//same
////    {
////        for (int i = 0; i < 12; ++i)
////        {
////            angle[i] = begin_angle[i] + (distance_same[i] - begin_angle[i]) * s1.getTCurve(count());
////        }
////    }
////    else if(prepose_=="symmetry")//symmetry
////    {
////        for (int i = 0; i < 12; ++i)
////        {
////            angle[i] = begin_angle[i] + (distance_symmetry[i] - begin_angle[i]) * s1.getTCurve(count());
////        }
////    }
////    else
////    {
////        mout() << "input error" << std::endl;
////    }
////
////    prepose = prepose_;
////
////    //输出角度，用于仿真测试
////    {
////        for (int i = 0; i < 12; ++i)
////        {
////            lout() << angle[i] << "\t";
////        }
////        time_test += 0.001;
////        lout() << time_test << "\t";
////        //输出身体和足尖曲线
////        for (int i = 0; i < 12; ++i)
////        {
////            lout() << file_current_leg[i] << "\t";
////        }
////        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
////    }
////    //发送电机角度
////    for (int i = 0; i < 12; ++i)
////    {
////        if (i == 2 || i == 5 || i == 8 || i == 11)
////            controller()->motionPool()[i].setTargetPos(   angle[i]);
////        else
////            controller()->motionPool()[i].setTargetPos(angle[i]);
////    }
////    return s1.getTc() * 1000 - count();
////}
////DogSwitchPre9,8,7,10,11,12};Pose::DogSwitchPrePose(const std::string& name)
////{
////    aris::core::fromXmlString(command(),
////        "<Command name=\"dog_switchpose\">"
////        "<Param name=\"prepose\" default=\"same\" abbreviation=\"p\"/>"
////        "</Command>");
////}
////DogSwitchPrePose::~DogSwitchPrePose() = default;
//

//轴空间准备
auto DogPrepare::prepareNrt()->void
{
    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogPrepare::executeRT()->int
{
    static double begin_angle[12]={0};
    double angle[12]={0};

    static double distance[12] = { 0 };
 
    for (int i = 0; i < 3; ++i)
        distance[i] = init_pos_angle[i];

    if (count() == 1)
    {
        this->master()->logFileRawName("prepare");
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        begin_angle[1] = controller()->motionPool()[1].actualPos();
        begin_angle[2] = controller()->motionPool()[2].actualPos();
//        begin_angle[3] = controller()->motionPool()[3].actualPos();
//        begin_angle[4] = controller()->motionPool()[4].actualPos();
//        begin_angle[5] = controller()->motionPool()[5].actualPos();
//        begin_angle[6] = controller()->motionPool()[6].actualPos();
//        begin_angle[7] = controller()->motionPool()[7].actualPos();
//        begin_angle[8] = controller()->motionPool()[8].actualPos();
//        begin_angle[9] = controller()->motionPool()[9].actualPos();
//        begin_angle[10] = controller()->motionPool()[10].actualPos();
//        begin_angle[11] = controller()->motionPool()[11].actualPos();
    }


    TCurve s1(5,2);//0.9s
    s1.getCurveParam();

    for(int i=0;i<3;++i)
    {
        angle[i] = begin_angle[i] + distance[i] * s1.getTCurve(count());
    }
    this->model()->setInputPos(angle);
    if (this->model()->forwardKinematics()) THROW_FILE_LINE("forward failed");
    
      
    //发送电机角度
    for (int i = 0; i < 3; ++i)
    {
        controller()->motionPool()[i].setTargetPos(angle[i]);
    }
    return s1.getTc() * 1000-count();
}
DogPrepare::DogPrepare(const std::string &name) 
{
    aris::core::fromXmlString(command(),
       "<Command name=\"dog_prepare\"/>");
}
DogPrepare::~DogPrepare() = default;

//下蹲起立  distance:上下移动的距离，输入正值为向上
auto DogUpDown::prepareNrt()->void
{
    distance_ = doubleParam("distance");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogUpDown::executeRT()->int
{
    if (count() == 1)this->master()->logFileRawName("updown");

    int ret = 1;

    TCurve s1(5, 2);//0.9s
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);

//    // test read toeque //
//    int16_t torque[12]={0};
//    for(int i=0;i<12;++i)
//    {
//        this->ecController()->motionPool()[i].readPdo(0x6077,0x00,torque[i]);
//        lout()<< torque[i] << "\t";
//    }
//    lout() << std::endl;


    //步态规划
    if (prepose == "same")// same
    {
        ret = updownPlanSameLeg(count() - 1, &e1, distance_, current_body_and_leg);
    }
    else if (prepose == "symmetry")  //symmetry
    {
        ret = updownPlanSymmetry(count() - 1, &e1, distance_, current_body_and_leg);
    }
    else //
    {
        mout() << "input error" << std::endl;
    }



    //计算逆运动学
    model()->setOutputPos(current_body_and_leg);
    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    for (auto m : model()->motionPool()) m.updP();
    model()->getInputPos(input_angle);

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << current_body_and_leg[i + 16] << "\t";
        }
        lout() << current_body_and_leg[3] << "\t" << current_body_and_leg[7] << "\t" << current_body_and_leg[11] << std::endl;
    }

    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(   input_angle[i]);
        else
            controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    return ret;
}
DogUpDown::DogUpDown(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_updown\">"
        "	<Param name=\"distance\" default=\"50\" abbreviation=\"d\"/>"
        "</Command>");
}
DogUpDown::~DogUpDown() = default;

//踏步
auto DogTaBu::prepareNrt()->void
{
    step_ = doubleParam("step");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogTaBu::executeRT()->int
{
    if (count() == 1)this->master()->logFileRawName("tabu");

    TCurve s1(1, 3);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0.1, 0, s1);

    int ret = 1;
    //步态规划
    if ( prepose == "same")// same
    {
        ret = trotPlanSameLeg(step_, count() - 1, &e1, current_body_and_leg);
    }
    else if (prepose == "symmetry")  //symmetry
    {
        ret = trotPlanSymmetryLeg(step_, count() - 1, &e1, current_body_and_leg);
    }
    else //
    {
        mout() << "input error" << std::endl;
    }

    //计算逆运动学
    model()->setOutputPos(current_body_and_leg);
    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    for (auto m : model()->motionPool()) m.updP();
    model()->getInputPos(input_angle);

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << current_body_and_leg[i+16] << "\t";
        }
        lout() << current_body_and_leg[3] << "\t" << current_body_and_leg[7] << "\t" << current_body_and_leg[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(   input_angle[i]);
        else
            controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    return ret;
}
DogTaBu::DogTaBu(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_tabu\">"
        "	<Param name=\"step\" default=\"1\" abbreviation=\"n\"/>"
        "</Command>");
}
DogTaBu::~DogTaBu() = default;

//前进
auto DogForward::prepareNrt()->void
{
    step_ = doubleParam("step");
    z_ = doubleParam("body_range");//前进是身体左右摇摆幅度
    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogForward::executeRT()->int
{

    if(count()==1)this->master()->logFileRawName("forward");

    int ret=0;
    TCurve s1(1, 3);
    s1.getCurveParam();
    EllipseTrajectory e1(0.2, 0.05, 0, s1);

    //步态规划
    if (gait == "trot" && prepose == "same")//trot & same
    {
        ret = trotPlanSameLeg(step_, count() - 1, &e1, current_body_and_leg);
    }
    else if (gait =="walk"  && prepose == "same") //walk & same
    {

        ret = walkPlanSameLeg(step_, count() - 1, &e1, current_body_and_leg);
        //ret = walkPlanSameLeg2(step_, count() - 1, &e1, current_body_and_leg,z_);
        //ret = walkPlanSameLeg3(step_, count() - 1, &e1, &s1,current_body_and_leg,body_cm);

    }
    else if (gait == "walk" && prepose == "symmetry")  //walk & symmetry
    {
        ret = walkPlanSymmetryLeg(step_, count() - 1, &e1, current_body_and_leg);
    }
    else //
    {
        mout() << "input error" << std::endl;
    }


    //计算逆运动学
    model()->setOutputPos(current_body_and_leg);
    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    for (auto m : model()->motionPool())
        m.updP();
    model()->getInputPos(input_angle);

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << current_body_and_leg[i + 16] << "\t";
        }
        lout() << current_body_and_leg[3] << "\t" << current_body_and_leg[7] << "\t" << current_body_and_leg[11] << std::endl;
        lout() << std::endl;
    }


//    //read joint angle to estimation
//    for (int i = 0; i < 3; ++i)
//    {
//        fk_input_joint_[i+9] = controller()->motionPool()[i].actualPos();

//    }
//    model()->setInputPos(fk_input_joint_);
//    if(model()->forwardKinematics())std::cout << "forward failed" << std::endl;
//    for(auto& gm : model()->generalMotionPool())
//        gm.updP();
//    model()->getOutputPos(leg_ee_);

    for(int i= 0;i<12;++i)
        joint_pos[i] = current_body_and_leg[i+16];
   // mout() << current_body_and_leg[25] << "\t" << current_body_and_leg[26] << "\t" << current_body_and_leg[27] << std::endl;
     //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
            controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }

    return ret;
}
DogForward::DogForward(const std::string &name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_forward\">"
             "<GroupParam>"
        "	<Param name=\"step\" default=\"1\" abbreviation=\"n\"/>"
        "	<Param name=\"body_range\" default=\"0.05\" abbreviation=\"z\"/>"
             "</GroupParam>"
        "</Command>");
}
DogForward::~DogForward() = default;

//后退`
auto DogBack::prepareNrt()->void
{
    step_ = doubleParam("step");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogBack::executeRT()->int
{
    if (count() == 1)this->master()->logFileRawName("back");
    int ret = 0;


    TCurve s1(1, 4);
    s1.getCurveParam();
    EllipseTrajectory e1(-0.15, 0.08, 0, s1);

    //步态规划
    if (gait == "trot" && prepose == "same")//trot & same
    {
        ret = trotPlanSameLeg(step_, count() - 1, &e1, current_body_and_leg);
    }
    else if (gait == "walk" && prepose == "same") //walk & same
    {
        ret = walkPlanSameLeg(step_, count() - 1, &e1, current_body_and_leg);
    }
    else if (gait == "walk" && prepose == "symmetry")  //walk & symmetry
    {
        ret = walkPlanSymmetryLeg(step_, count() - 1, &e1, current_body_and_leg);
    }
    else //
    {
        mout() << "input error" << std::endl;
    }


    //计算逆运动学
    model()->setOutputPos(current_body_and_leg);
    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    for (auto m : model()->motionPool())
        m.updP();
    model()->getInputPos(input_angle);

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << current_body_and_leg[i + 16] << "\t";
        }
        lout() << current_body_and_leg[3] << "\t" << current_body_and_leg[7] << "\t" << current_body_and_leg[11] << std::endl;
    }

    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(   input_angle[i]);
        else
            controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    return ret;
}
DogBack::DogBack(const std::string& name)
{

    aris::core::fromXmlString(command(),
        "<Command name=\"dog_back\">"
        "	<Param name=\"step\" default=\"1\" abbreviation=\"n\"/>"
        "</Command>");
}
DogBack::~DogBack() = default;

//左移
auto DogLeft::prepareNrt()->void
{
    step_ = doubleParam("step");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogLeft::executeRT()->int
{
    int ret = 0;

    //轨迹规划
    TCurve s1(1, 4);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0.08, -0.1, s1);

    //步态规划
    if (gait == "trot" && prepose == "same")//trot & same
    {
        ret = trotPlanSameLeg(step_, count() - 1, &e1, current_body_and_leg);
    }
    else if (gait == "walk" && prepose == "same") //walk & same
    {
        ret = walkPlanSameLeg(step_, count() - 1, &e1, current_body_and_leg);
    }
    else if (gait == "walk" && prepose == "symmetry")  //walk & symmetry
    {
        ret = walkPlanSymmetryLeg(step_, count() - 1, &e1, current_body_and_leg);
    }
    else //
    {
        mout() << "input error" << std::endl;
    }


    //计算逆运动学
    model()->setOutputPos(current_body_and_leg);
    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    for (auto m : model()->motionPool())
        m.updP();
    model()->getInputPos(input_angle);

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << current_body_and_leg[i + 16] << "\t";
        }
        lout() << current_body_and_leg[3] << "\t" << current_body_and_leg[7] << "\t" << current_body_and_leg[11] << std::endl;
    }

    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    return ret;
}
DogLeft::DogLeft(const std::string& name)
{
        aris::core::fromXmlString(command(),
            "<Command name=\"dog_left\">"
            "	<Param name=\"step\" default=\"1\" abbreviation=\"n\"/>"
            "</Command>");
}
DogLeft::~DogLeft() = default;

//右移
auto DogRight::prepareNrt()->void
{
    step_ = doubleParam("step");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogRight::executeRT()->int
{
    int ret = 0;
    if (count() == 1)  this->master()->logFileRawName("right");

    TCurve s1(1, 4);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0.08,0.1, s1);

    //步态规划
    if (gait == "trot" && prepose == "same")//trot & same
    {
        ret = trotPlanSameLeg(step_, count() - 1, &e1, current_body_and_leg);
    }
    else if (gait == "walk" && prepose == "same") //walk & same
    {
        ret = walkPlanSameLeg(step_, count() - 1, &e1, current_body_and_leg);
    }
    else if (gait == "walk" && prepose == "symmetry")  //walk & symmetry
    {
        ret = walkPlanSymmetryLeg(step_, count() - 1, &e1, current_body_and_leg);
    }
    else //
    {
        mout() << "input error" << std::endl;
    }


    //计算逆运动学
    model()->setOutputPos(current_body_and_leg);
    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    for (auto m : model()->motionPool())
        m.updP();
    model()->getInputPos(input_angle);

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << current_body_and_leg[i + 16] << "\t";
        }
        lout() << current_body_and_leg[3] << "\t" << current_body_and_leg[7] << "\t" << current_body_and_leg[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    return ret;
}
DogRight::DogRight(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_right\">"
        "	<Param name=\"step\" default=\"1\" abbreviation=\"n\"/>"
        "</Command>");
}
DogRight::~DogRight() = default;

//原地旋转
auto DogTurn::prepareNrt()->void

{
    step_ = doubleParam("step");
    turn_angle_ = doubleParam("angle");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogTurn::executeRT()->int
{
    int ret = 0;
    if (count() == 1)  this->master()->logFileRawName("right");

    TCurve s1(1, 6);
    s1.getCurveParam();
    EllipseTrajectory e1(0.0, 0.015, 0.0, s1);
    BodyPose body_s(0, turn_angle_, 0, s1);
    //步态规划
    if (gait == "trot" && prepose == "same")//trot & same
    {
        ret = turnPlanTrotSameLeg(step_, count() - 1, &e1, &body_s, current_body_and_leg);
    }
    else //
    {
        mout() << "input error" << std::endl;
    }


    //计算逆运动学
    model()->setOutputPos(current_body_and_leg);
    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    for (auto m : model()->motionPool())
        m.updP();
    model()->getInputPos(input_angle);

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << current_body_and_leg[i + 16] << "\t";
        }
        lout() << current_body_and_leg[3] << "\t" << current_body_and_leg[7] << "\t" << current_body_and_leg[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(   input_angle[i]);
        else
            controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }

    return ret;
}
DogTurn::DogTurn(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_turn\">"
        "<GroupParam>"
        "<Param name=\"step\" default=\"3\" abbreviation=\"n\"/>"
        "<Param name = \"angle\" default=\"10\" abbreviation=\"d\"/>"
        "</GroupParam>"
        "</Command>");
}
DogTurn::~DogTurn() = default;





//pitch
auto DogPitch::prepareNrt()->void
{
    turn_angle_ = doubleParam("angle");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogPitch::executeRT()->int
{
    if (count() == 1)
    {
        mout() << "turn_angle_ = " << turn_angle_ << std::endl;
        this->master()->logFileRawName("pitch");
    }

    int ret = 0;


    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);
    BodyPose body_pose(0, 0, turn_angle_, s1);
    //步态规划 inverseSame(current_leg_in_ground, current_body_in_ground, input);

    if (std::abs(turn_angle_) > 10)
    {
        mout() << "input exceed mas value" << std::endl;
        return 0;
    }
    else
    {
        ret = posePlan(count() - 1, &e1, &body_pose, current_body_and_leg);
    }


    //计算逆运动学
    model()->setOutputPos(current_body_and_leg);
    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    for (auto m : model()->motionPool())
        m.updP();
    model()->getInputPos(input_angle);

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << current_body_and_leg[i + 16] << "\t";
        }
        lout() << current_body_and_leg[3] << "\t" << current_body_and_leg[7] << "\t" << current_body_and_leg[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(   input_angle[i]);
        else
            controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    return ret;
}
DogPitch::DogPitch(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_pitch\">"
        "	<Param name=\"angle\" default=\"5\" abbreviation=\"d\"/>"
        "</Command>");
}
DogPitch::~DogPitch() = default;

//roll
auto DogRoll::prepareNrt()->void
{
    turn_angle_ = doubleParam("angle");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogRoll::executeRT()->int
{
    if (count() == 1)
    {
         mout() << "turn_angle_ = " << turn_angle_ << std::endl;
         this->master()->logFileRawName("roll");
    }
    int ret = 0;


    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);
    BodyPose body_pose(turn_angle_, 0, 0, s1);
    //步态规划
    if (std::abs(turn_angle_) > 10)
    {
        mout() << "input exceed mas value" << std::endl;
        return 0;
    }
    else
    {
        ret = posePlan(count() - 1, &e1, &body_pose, current_body_and_leg);
    }

    //计算逆运动学
    model()->setOutputPos(current_body_and_leg);
    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    for (auto m : model()->motionPool())
        m.updP();
    model()->getInputPos(input_angle);

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << current_body_and_leg[i + 16] << "\t";
        }
        lout() << current_body_and_leg[3] << "\t" << current_body_and_leg[7] << "\t" << current_body_and_leg[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)

            controller()->motionPool()[i].setTargetPos(   input_angle[i]);
        else
            controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    return ret;
}
DogRoll::DogRoll(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_roll\">"
        "	<Param name=\"angle\" default=\"5\" abbreviation=\"d\"/>"
        "</Command>");
}
DogRoll::~DogRoll() = default;


//yaw
auto DogYaw::prepareNrt()->void
{
    turn_angle_ = doubleParam("angle");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogYaw::executeRT()->int
{
    if (count() == 1)
    {
        mout() << "turn_angle_ = " << turn_angle_ << std::endl;
        this->master()->logFileRawName("yaw");
    }
    int ret = 0;


    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);
    BodyPose body_pose(0, turn_angle_, 0, s1);
    //步态规划 inverseSame(current_leg_in_ground, current_body_in_ground, input);


    if (std::abs(turn_angle_) > 30)
    {
        mout() << "input exceed mas value" << std::endl;
        return 0;
    }
    else
    {
        ret = posePlan(count() - 1, &e1, &body_pose, current_body_and_leg);
    }

    //计算逆运动学
    model()->setOutputPos(current_body_and_leg);
    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    for (auto m : model()->motionPool())
        m.updP();
    model()->getInputPos(input_angle);

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << current_body_and_leg[i + 16] << "\t";
        }
        lout() << current_body_and_leg[3] << "\t" << current_body_and_leg[7] << "\t" << current_body_and_leg[11] << std::endl;
    }

    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(   input_angle[i]);
        else
            controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    return ret;
}
DogYaw::DogYaw(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_yaw\">"
        "	<Param name=\"angle\" default=\"15\" abbreviation=\"d\"/>"
        "</Command>");
}
DogYaw::~DogYaw() = default;


// pose
auto DogPose::prepareNrt()->void
{
    turn_angle_ = doubleParam("angle");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogPose::executeRT()->int
{
    if (count() == 1)
    {
        mout() << "turn_angle_ = " << turn_angle_ << std::endl;
        this->master()->logFileRawName("pose");
    }
    int ret = 0;


    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);
    static double pose_angle[3] = {0, 0, 0};  // 321  zyx
//    pose_angle[0] = gamepad[0] * 10.0 * PI / 180;
//    pose_angle[1] = gamepad[1] * 10.0 * PI / 180;


    pose_angle[0] = pose_angle[0] * (1 - 0.1) + (gamepad[0] * 10.0 * PI / 180) * 0.1;
    pose_angle[1] = pose_angle[1] * (1 - 0.1) + (gamepad[1] * 10.0 * PI / 180) * 0.1;
    pose_angle[2] = 0;
    if (std::abs(turn_angle_) > 30)
    {
        mout() << "input exceed mas value" << std::endl;
        return 0;
    }
    else
    {
        ret = posePlan(pose_angle, current_body_and_leg);
    }

    if(count()% 100 == 0)
        aris::dynamic::dsp(4,4,current_body_and_leg);
    //计算逆运动学
    model()->setOutputPos(current_body_and_leg);
    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    for (auto m : model()->motionPool())
        m.updP();
    model()->getInputPos(input_angle);

//    //输出角度，用于仿真测试
//    {
//        for (int i = 0; i < 12; ++i)
//        {
//            lout() << input_angle[i] << "\t";
//        }

//        //输出身体和足尖曲线
//        for (int i = 0; i < 12; ++i)
//        {
//            lout() << current_body_and_leg[i + 16] << "\t";
//        }
//        lout() << current_body_and_leg[3] << "\t" << current_body_and_leg[7] << "\t" << current_body_and_leg[11] << std::endl;
//    }

    //发送电机角度
//    for (int i = 0; i < 3; ++i)
//    {
//        controller()->motionPool()[i].setTargetPos(input_angle[i]);

//    }
    return 1;
}
DogPose::DogPose(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_pose\">"
        "	<Param name=\"angle\" default=\"15\" abbreviation=\"d\"/>"
        "</Command>");
}
DogPose::~DogPose() = default;

// cpp和adams测试 //
auto DogDynamicTest::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogDynamicTest::executeRT()->int
{
    //if (count() == 1) this->master()->logFileRawName("dynamic");

    int ret = 0,a= 1000;
    static double ee0[28];
    double ee_p[28];
    double ee_v[18] = { 0 };
    double ee_a[18] = { 0 };
    double mf[12];
    //static double pre_leg_pos[28] = 
    //{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    //  0.30199000000000,-0.52999937168147,-0.19700000000000,
    // -0.30199000000000,-0.53000000000000,-0.19700000000000,
    // -0.30199000000000,-0.52999937168147, 0.19700000000000,
    //  0.30199000000000,-0.53000000000000, 0.19700000000000 };
    static double pre_leg_vel[28] = { 0 };

    //--------------------------运动学测试-------------------------------------//

    if (count() <= a)
    {
        ret = 1;
        //-----------------------规划轨迹-----------------------------// 
        if (count() == 1) {
            model()->getOutputPos(ee0);
            aris::dynamic::s_vc(28, ee0, ee_p);
        }
        aris::dynamic::s_vc(28, ee0, ee_p + 0);
        model()->setOutputPos(ee_p);
        if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
        model()->setTime(0.001 * count());
    }
    else
    {
       //
        //ret =  0;
        //-----------------------规划轨迹-----------------------------// 
       
        TCurve s1(5,2 );
        s1.getCurveParam();
        EllipseTrajectory e1(0.20, 0.080, 0, s1);
        EllipseTrajectory e2(0, 0, 0, s1);
        EllipseTrajectory e3(0, 0.150, 0, s1);
        BodyPose body_s(0, 15, 0, s1);
        // 前后左右 //
        //ret = trotPlanSameLeg(10, count() - 1-a, &e1, current_body_and_leg);
        //ret = walkPlanSameLeg(10, count() - 1-a, &e1, current_body_and_leg);
        //ret = walkPlanSymmetryLeg(5, count() - 1 - a, &e1, current_body_and_leg);
        // 原地旋转 //
        ret = turnPlanTrotSameLeg(5, count() - 1- a, &e3, &body_s, current_body_and_leg);
        // 身子扭动 //
        //ret = posePlan(count() - 1-a, &e2, &body_s, current_body_and_leg);

        model()->setOutputPos(current_body_and_leg);
        if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;




        model()->setTime(0.001 * count());
        if (ret == 1)
            std::cout << s1.getTc() * 1000 << std::endl;
        if (ret == 0) std::cout << count() << std::endl;
    }

    //---------------------------------静力学----------------------------------------//

    //if (count() < a) //再初始状态下保持不动
    //{
    //    ret = 1;
    //   // // 身体固定，有效，足尖无约束 //
    //   // if (count() == 1) 
    //   // {
    //   //     model()->getOutputPos(ee0);
    //   //     aris::dynamic::s_vc(28, ee0, ee_p);
    //   // }
    //   // aris::dynamic::s_vc(28, ee0, ee_p + 0);
    //   // // 设置末端轨迹，计算运动学反解 //
    //   // model()->setOutputPos(ee_p);
    //   // if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    //   // for (auto& gm : model()->generalMotionPool())
    //   // {
    //   //     gm.updP(); gm.updV();
    //   // }
    //   // if (model()->inverseKinematicsVel())std::cout << "inverse vel failed" << std::endl;
    //   // // 站立情况下的足尖力，从adams接触中读的 //
    //   // double f_init[4][6] =
    //   // {
    //   //     {0,67,0,13.8,0,37.7},
    //   //     {0,84,0,17.2,0,3.38},
    //   //     {0,82,0,-16.81,0,3.31},
    //   //     {0,69,0,-14.13,0,38.8},
    //   // };
    //   // double f[4][6] = { 0 };
    //   // double pm_ee1[16], pm_ee2[16],pm_ee3[16],pm_ee4[16];   
    //   // model()->generalMotionPool()[1].makI()->getPm(pm_ee1);
    //   // model()->generalMotionPool()[2].makI()->getPm(pm_ee2);
    //   // model()->generalMotionPool()[3].makI()->getPm(pm_ee3);
    //   // model()->generalMotionPool()[4].makI()->getPm(pm_ee4);
    //   // aris::dynamic::s_tf(pm_ee1, f_init[0], f[0]);
    //   // aris::dynamic::s_tf(pm_ee2, f_init[1], f[1]);
    //   // aris::dynamic::s_tf(pm_ee3, f_init[2], f[2]);
    //   // aris::dynamic::s_tf(pm_ee4, f_init[3], f[3]);
    //   //dynamic_cast <aris::dynamic::GeneralForce&> (model()->forcePool()[0]).setFce(f[0]);
    //   //dynamic_cast <aris::dynamic::GeneralForce&> (model()->forcePool()[1]).setFce(f[1]);
    //   //dynamic_cast <aris::dynamic::GeneralForce&> (model()->forcePool()[2]).setFce(f[2]);
    //   //dynamic_cast <aris::dynamic::GeneralForce&> (model()->forcePool()[3]).setFce(f[3]);
    //   // model()->solverPool()[4].dynAccAndFce();
    //   // double mf[12];
    //   // model()->getInputFce(mf);
    //   // if (count() == 1)
    //   // {
    //   //     std::cout << "mf" << std::endl;
    //   //     aris::dynamic::dsp(4, 3, mf);
    //   // }
    //   // model()->setTime(0.001 * count());
    //    // 身体失效，脚尖为球铰约束 //
    //    if (count() == 1)
    //    {
    //        model()->getOutputPos(ee0);
    //        aris::dynamic::s_vc(28, ee0, ee_p);
    //    }
    //    aris::dynamic::s_vc(28, ee0, ee_p + 0);
    //    model()->setOutputPos(ee_p);
    //    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    //    for (auto& gm : model()->generalMotionPool())
    //    {
    //        gm.updP(); gm.updV();
    //    }
    //    model()->setOutputVel(ee_v);
    //    if (model()->inverseKinematicsVel())std::cout << "inverse vel failed" << std::endl;
    //    model()->setOutputAcc(ee_a);
    //    double f_init[4][6] =
    //    {
    //        {0,67,0,13.8,0,37.7},
    //        {0,84,0,17.2,0,3.38},
    //        {0,82,163840,-16.81,0,3.31},
    //        {0,69,0,-14.13,0,38.8},
    //    };
    //    double f[4][6] = { 0 };
    //    double pm_ee1[16], pm_ee2[16], pm_ee3[16], pm_ee4[16];
    //    model()->generalMotionPool()[1].makI()->getPm(pm_ee1);
    //    model()->generalMotionPool()[2].makI()->getPm(pm_ee2);
    //    model()->generalMotionPool()[3].makI()->getPm(pm_ee3);
    //    model()->generalMotionPool()[4].makI()->getPm(pm_ee4);
    //    aris::dynamic::s_tf(pm_ee1, f_init[0], f[0]);
    //    aris::dynamic::s_tf(pm_ee2, f_init[1], f[1]);
    //    aris::dynamic::s_tf(pm_ee3, f_init[2], f[2]);
    //    aris::dynamic::s_tf(pm_ee4, f_init[3], f[3]);
    //    dynamic_cast <aris::dynamic::GeneralForce&> (model()->forcePool()[0]).setFce(f[0]);
    //    dynamic_cast <aris::dynamic::GeneralForce&> (model()->forcePool()[1]).setFce(f[1]);
    //    dynamic_cast <aris::dynamic::GeneralForce&> (model()->forcePool()[2]).setFce(f[2]);
    //    dynamic_cast <aris::dynamic::GeneralForce&> (model()->forcePool()[3]).setFce(f[3]);
    //    model()->solverPool()[5].dynAccAndFce();
    //    double mf[12];
    //    model()->getInputFce(mf);
    //    if (count() == 100)
    //    {
    //        //auto& s = dynamic_cast<aris::dynamic::UniversalSolver&>(model()->solverPool()[4]);
    //        //s.cptGeneralInverseDynamicMatrix();
    //        //dsp(s.nM(), s.nM(), s.M());
    //        //dsp(1, s.nM(), s.h());
    //        //std::cout << "n:" << s.nM() <<  std::endl;
    //        //double a[24];
    //        //model()->getInputAcc(a);
    //        //model()->getOutputAcc(a + 12);
    //        //std::cout << model()->outputDim() <<std::endl;
    //        //dsp(1, s.nM(), a);
    //        //
    //        //dsp(1, 3, model()->generalMotionPool()[1].a());
    //        
    //        std::cout << "mf" << std::endl;
    //        aris::dynamic::dsp(4, 3, mf);
    //    }
    //    model()->setTime(0.001 * count());
    //}
    //else // 行走或转动时
    //{
    //   ret = 0;
    //    model()->setTime(0.001 * count());
    //}



    //----------------------------------动力学---------------------------------//

    //if (count() <= a) //再初始状态下保持不动
    //{
    //    ret = 1;
    //    // 身体固定，有效，足尖无约束 //
    //    if (count() == 1)
    //    {
    //        model()->getOutputPos(ee0);
    //        aris::dynamic::s_vc(28, ee0, ee_p);
    //    }
    //    aris::dynamic::s_vc(28, ee0, ee_p + 0);
//
    //    for (int i = 0; i < 12; ++i)
    //    {
    //        ee_v[i + 6] = 0;
    //        ee_a[i + 6] = 0;
    //    }
//
    //    model()->setOutputPos(ee_p);
    //    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    //    for (auto& gm : model()->generalMotionPool()) { gm.updP(); gm.updV(); }
    //    for (auto& m : model()->motionPool()) { m.updP(); m.updV(); }
    //    model()->setOutputVel(ee_v);
    //    if (model()->inverseKinematicsVel())std::cout << "inverse vel failed" << std::endl;
    //    for (auto& gm : model()->generalMotionPool()) { gm.updP(); gm.updV(); }
    //    for (auto& m : model()->motionPool()) { m.updP(); m.updV(); }
    //    model()->setOutputAcc(ee_a);
    //    model()->solverPool()[0].dynAccAndFce();
    //    for (auto& m : model()->motionPool()) { m.updA(); }
    //    double ma[18];
    //    model()->getInputAcc(ma);
    //    model()->solverPool()[4].dynAccAndFce();
    //    model()->getInputFce(mf);
//
    //    if (count() == 1)
    //        aris::dynamic::dsp(4, 3, mf);
    //    model()->setTime(0.001 * count());
    //    // 身体失效，脚尖为球铰约束 //
    //    // 规划轨迹 // 
    //    //if (count() == 1) 
    //    //{
    //    //    model()->getOutputPos(ee0);
    //    //    aris::dynamic::s_vc(28, ee0, ee_p);
    //    //}
    //    //aris::dynamic::s_vc(28, ee0, ee_p + 0);
    //    //// 设置末端轨迹，计算运动学反解 //
    //    //model()->setOutputPos(ee_p);
    //    //if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    //    //for (auto& gm : model()->generalMotionPool())
    //    //{
    //    //    gm.updP(); gm.updV();
    //    //}
    //    //if (model()->inverseKinematicsVel())std::cout << "inverse vel failed" << std::endl;
    //    //model()->getOutputVel(ee_v);
    //    //if (count() == 1)
    //    //{
    //    //    aris::dynamic::dsp(4, 4, ee_v);
    //    //    aris::dynamic::dsp(4, 3, ee_v + 16);
    //    //}
    //    //model()->solverPool()[4].dynAccAndFce();
    //    //double mf[12];
    //    //model()->getInputFce(mf);
    //    //if(count()==1)
    //    //aris::dynamic::dsp(4, 3, mf);
    //    //model()->setTime(0.001 * count());
    //}
    //else // 行走或转动时
    //{
    //    // 身体固定不动 // 
    //    TCurve s1(5,2 );
    //    s1.getCurveParam();
    //    EllipseTrajectory e1(0.150, 0.080, 0, s1);
    //    EllipseTrajectory e2(0, 0, 0, s1);
    //    EllipseTrajectory e3(0, 0.150, 0, s1);
    //    BodyPose body_s(0, 15, 0, s1);
    //    // 前后左右 //
    //    ret = trotPlanSameLeg(1, count() - 1-a, &e1, input_angle);
    //    //ret = walkPlanSameLeg(1, count() - 1-a, &e1, input_angle);
    //    //ret = walkPlanSymmetryLeg(5, count() - 1 - a, &e1, input_angle);
    //    // 原地旋转 //
    //    //ret = turnPlanTrotSameLeg(5, count() - 1- a, &e3, &body_s, input_angle);
    //    // 身子扭动 //
    //    //ret = posePlan(count() - 1-a, &e2, &body_s, input_angle);
    //    double sss[16] = { 1,0,0,0,
    //                        0,1,0,0,
    //                        0,0,1,0,
    //                        0,0,0,1 };
    //    aris::dynamic::s_vc(16, sss + 0, ee_p + 0);
    //    aris::dynamic::s_vc(12, file_current_leg + 0, ee_p + 16);
   //
    //    for (int i = 0; i < 12; ++i)
    //    {
    //         ee_v[i+6] = (pre_leg_pos[i+16] - file_current_leg[i]) * 1000.0;
    //        ee_a[i+6] = (pre_leg_vel[i+6] - ee_v[i+6]) * 1000.0;
    //        pre_leg_pos[i + 16] = file_current_leg[i];
    //        pre_leg_vel[i + 6] = ee_v[i + 6];
    //    }
    //    
    //   
    //    model()->setOutputPos(ee_p);
    //    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
    //    for (auto& gm : model()->generalMotionPool()){ gm.updP(); gm.updV(); }
    //    for (auto& m : model()->motionPool()){ m.updP(); m.updV(); } 
    //    model()->setOutputVel(ee_v);
    //    if (model()->inverseKinematicsVel())std::cout << "inverse vel failed" << std::endl;
    //    for (auto& gm : model()->generalMotionPool()) { gm.updP(); gm.updV(); }
    //    for (auto& m : model()->motionPool()) { m.updP(); m.updV(); }
    //    model()->setOutputAcc(ee_a);
    //    model()->solverPool()[0].dynAccAndFce();
    //    for (auto& m : model()->motionPool()) { m.updA(); }
    //    double ma[18];
    //    model()->getInputAcc(ma);
    //    model()->solverPool()[4].dynAccAndFce();
    //    model()->getInputFce(mf);
//
    //    if (count() == 1200)
    //    {
    //        std::cout << "mf" << std::endl;
    //        aris::dynamic::dsp(4, 3, mf);
    //    }
    //    model()->setTime(0.001 * count());
    //}
    //
    
    //for (int i = 0; i < 12; ++i)
    //    lout() << mf[i] << "\t" ;
    //lout() << std::endl;


    if (ret == 0) std::cout << count() << std::endl;
    return ret;
}
DogDynamicTest::DogDynamicTest(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_dynamic\"/>");

}
DogDynamicTest::~DogDynamicTest() = default;


// run plot test //
auto DogRun::prepareNrt()->void
{
    step_ = doubleParam("step");
    z_ = doubleParam("body_range");//前进是身体左右摇摆幅度
    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogRun::executeRT()->int
{
    static double begin_angle[12]={0};
    double angle[12]={0};

    if (count() == 1)this->master()->logFileRawName("prepare");

    static double distance[12] = { 0 };

    for (int i = 0; i < 12; ++i)
        distance[i] = init_pos_angle[i];

    if (count() == 1)
    {
        this->master()->logFileRawName("prepare");
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        begin_angle[1] = controller()->motionPool()[1].actualPos();
        begin_angle[2] = controller()->motionPool()[2].actualPos();
        begin_angle[3] = controller()->motionPool()[3].actualPos();
        begin_angle[4] = controller()->motionPool()[4].actualPos();
        begin_angle[5] = controller()->motionPool()[5].actualPos();
        begin_angle[6] = controller()->motionPool()[6].actualPos();
        begin_angle[7] = controller()->motionPool()[7].actualPos();
        begin_angle[8] = controller()->motionPool()[8].actualPos();
        begin_angle[9] = controller()->motionPool()[9].actualPos();
        begin_angle[10] = controller()->motionPool()[10].actualPos();
        begin_angle[11] = controller()->motionPool()[11].actualPos();
    }


    TCurve s1(5,2);//0.9s
    s1.getCurveParam();

    for(int i=0;i<12;++i)
    {
        angle[i] = begin_angle[i] + distance[i] * s1.getTCurve(count());
        joint_pos[i] = begin_angle[i] + distance[i] * s1.getTCurve(count());
    }
    this->model()->setInputPos(angle);
    if (this->model()->forwardKinematics()) THROW_FILE_LINE("forward failed");


    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(   angle[i]);
        else
            controller()->motionPool()[i].setTargetPos(angle[i]);
    }
    return s1.getTc() * 1000-count();


//    if(count()==1)this->master()->logFileRawName("forward");

//    int ret=0;
//    TCurve s1(1, 3);
//    s1.getCurveParam();
//    EllipseTrajectory e1(0.2, 0.05, 0, s1);

//    //步态规划
//    if (gait == "trot" && prepose == "same")//trot & same
//    {
//        ret = trotPlanSameLeg(step_, count() - 1, &e1, current_body_and_leg);
//    }
//    else if (gait =="walk"  && prepose == "same") //walk & same
//    {

//        ret = walkPlanSameLeg(step_, count() - 1, &e1, current_body_and_leg);
//        //ret = walkPlanSameLeg2(step_, count() - 1, &e1, current_body_and_leg,z_);
//        //ret = walkPlanSameLeg3(step_, count() - 1, &e1, &s1,current_body_and_leg,body_cm);

//    }
//    else if (gait == "walk" && prepose == "symmetry")  //walk & symmetry
//    {
//        ret = walkPlanSymmetryLeg(step_, count() - 1, &e1, current_body_and_leg);
//    }
//    else //
//    {
//        mout() << "input error" << std::endl;
//    }


//    //计算逆运动学
//    model()->setOutputPos(current_body_and_leg);
//    if (model()->inverseKinematics())std::cout << "inverse failed" << std::endl;
//    for (auto m : model()->motionPool())
//        m.updP();
//    model()->getInputPos(input_angle);

//    //输出角度，用于仿真测试
//    {
//        for (int i = 0; i < 12; ++i)
//        {
//            lout() << input_angle[i] << "\t";
//        }

//        //输出身体和足尖曲线
//        for (int i = 0; i < 12; ++i)
//        {
//            lout() << current_body_and_leg[i + 16] << "\t";
//        }
//        lout() << current_body_and_leg[3] << "\t" << current_body_and_leg[7] << "\t" << current_body_and_leg[11] << std::endl;

//    }
//    lout() << std::endl;

//    //发送电机角度
//    for (int i = 0; i < 12; ++i)
//    {
//        if (i == 2 || i == 5 || i == 8 || i == 11)
//            controller()->motionPool()[i].setTargetPos(   input_angle[i]);
//        else
//            controller()->motionPool()[i].setTargetPos(input_angle[i]);
//    }

//    return ret;
}
DogRun::DogRun(const std::string &name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_run\">"
             "<GroupParam>"
        "	<Param name=\"step\" default=\"1\" abbreviation=\"n\"/>"
        "	<Param name=\"body_range\" default=\"0.05\" abbreviation=\"z\"/>"
             "</GroupParam>"
        "</Command>");
}
DogRun::~DogRun() = default;



class Quad :public aris::dynamic::Model
{
public:
    auto virtual init()->void override
    {
        this->Model::init();
        
        
        // 设置身体固定时的拓扑结构 —— 静力学 // 
        for (auto& m  : this->motionPool())m.activate(true);
        for (auto& gm : this->generalMotionPool())gm.activate(false);
        for (auto& f : this->forcePool())f.activate(false);
        this->generalMotionPool()[0].activate(true);
        this->solverPool()[4].allocateMemory();

        // 站立时的拓扑结构 //
        for (auto& m : this->motionPool())m.activate(true);
        for (auto& gm : this->generalMotionPool())gm.activate(true);
        for (auto& f : this->forcePool())f.activate(false);
        this->generalMotionPool()[0].activate(false);
        this->solverPool()[5].allocateMemory();

        // 其他尝试
        for (auto& m : this->motionPool())m.activate(true);
        for (auto& gm : this->generalMotionPool())gm.activate(false);
        for (auto& f : this->forcePool())f.activate(false);
        this->generalMotionPool()[0].activate(false);
        this->solverPool()[6].allocateMemory();

    }

    Quad() 
    {
        // set gravity //
        const double gravity[6]{ 0.0,-9.8,0.0,0.0,0.0,0.0 };

        this->environment().setGravity(gravity);

        //define joint pos //
        const double leg_pe[12][3]{
            {  kBodyLong / 2,   0, -kBodyWidth / 2      },
            {  kBodyLong / 2,   0, -kBodyWidth / 2 - L1_new_motor },
            {  kBodyLong / 2, -L2, -kBodyWidth / 2 - L1_new_motor },
            { -kBodyLong / 2,   0, -kBodyWidth / 2      },
            { -kBodyLong / 2,   0, -kBodyWidth / 2 - L1 },
            { -kBodyLong / 2, -L2, -kBodyWidth / 2 - L1 },
            { -kBodyLong / 2,   0,  kBodyWidth / 2      },
            { -kBodyLong / 2,   0,  kBodyWidth / 2 + L1 },
            { -kBodyLong / 2, -L2,  kBodyWidth / 2 + L1 },
            {  kBodyLong / 2,   0,  kBodyWidth / 2      },
            {  kBodyLong / 2,   0,  kBodyWidth / 2 + L1_new_motor },
            {  kBodyLong / 2, -L2,  kBodyWidth / 2 + L1_new_motor },
        };
        //define ee pos //
        const double ee_pos[4][6]{
        { kBodyLong / 2, -L3 - L2, -(kBodyWidth / 2) - L1,0.0, 0.0, 0.0},   //leg1 ->012
        {-kBodyLong / 2, -L3 - L2, -(kBodyWidth / 2) - L1,0.0, 0.0, 0.0},   //leg2 ->345
        {-kBodyLong / 2, -L3 - L2,  (kBodyWidth / 2) + L1,0.0, 0.0, 0.0},   //leg3 ->678
        { kBodyLong / 2, -L3 - L2,  (kBodyWidth / 2) + L1,0.0, 0.0, 0.0},   //leg4 ->91011
        };

        ////iv:  10x1 惯量矩阵向量[m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
        //// define iv param //  材料都为铝 总重25.037kg  第二代
        //const double body_iv[10]{ 11.244602964,0,0,0,0.6510608871,0.5989223304,7.9786361064E-02,0,0,0 };
        ////leg1
        //const double lf_p1_iv[10]{ 2.1374710038,0,0,0,5.5931718868E-03,5.472611078E-03,2.6340502655E-03,0,0,0 };
        //const double lf_p2_iv[10]{ 0.4968154968,0,0,0,4.3644561758E-03,4.270698531E-03,2.3175168139E-04,0,0,0 };
        //const double lf_p3_iv[10]{ 0.8145237429,0,0,0,1.0780253535E-02,1.076892825E-02,03.695341839E-04,0,0,0 };
        ////leg2
        //const double lr_p1_iv[10]{ 2.1374710038,0,0,0,5.5931718868E-03,5.472611078E-03,2.6340502655E-03,0,0,0 };
        //const double lr_p2_iv[10]{ 0.4968154968,0,0,0,4.3644561758E-03,4.270698531E-03,2.3175168139E-04,0,0,0 };
        //const double lr_p3_iv[10]{ 0.8145237429,0,0,0,1.0780253535E-02,1.076892825E-02,03.695341839E-04,0,0,0 };
        ////leg3
        //const double rr_p1_iv[10]{ 2.1374710037,0,0,0,5.5931718868E-03,5.4726110785E-03,2.6340502654E-03,0,0,0 };
        //const double rr_p2_iv[10]{ 0.4968154968,0,0,0,4.3644561728E-03,4.2706985298E-03,2.3175167966E-04,0,0,0 };
        //const double rr_p3_iv[10]{ 0.814523742,0,0,0,1.0780253649E-02,1.0768928365E-02,3.6953418362E-04,0,0,0 };
        ////leg4
        //const double rf_p1_iv[10]{ 2.1374710038,0,0,0,5.5931718868E-03,5.472611078E-03,2.6340502655E-03,0,0,0 };
        //const double rf_p2_iv[10]{ 0.4968154968,0,0,0,4.3644561758E-03,4.270698531E-03,2.3175168139E-04,0,0,0 };
        //const double rf_p3_iv[10]{ 0.8145237429,0,0,0,1.0780253535E-02,1.076892825E-02,03.695341839E-04,0,0,0 };


        //iv:  10x1 惯量矩阵向量[m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
        // define iv param //  材料都为铝 总重30.86kg  第2.5代
        const double body_iv[10]{ 15.1007177439,0,0,0,0.68976308,0.612989762,0.1389151407,0,0,0 };
        //leg1
        const double lf_p1_iv[10]{ 2.3523814491,0,0,0,1.0480809884E-02,8.8825903772E-03,4.266754639E-03,0,0,0 };
        const double lf_p2_iv[10]{ 0.5868045476,0,0,0,6.9738871315E-03,6.8932585512E-03,5.6065308443E-04,0,0,0 };
        const double lf_p3_iv[10]{ 1.0111367876,0,0,0,1.1955933401E-02,1.1656650714E-02,5.8819775854E-04,0,0,0 };
        //leg2
        const double lr_p1_iv[10]{ 2.3523814491,0,0,0,1.0480809884E-02,8.8825903772E-03,4.266754639E-03,0,0,0 };
        const double lr_p2_iv[10]{ 0.5868045476,0,0,0,6.9738871315E-03,6.8932585512E-03,5.6065308443E-04,0,0,0 };
        const double lr_p3_iv[10]{ 1.0111367876,0,0,0,1.1955933401E-02,1.1656650714E-02,5.8819775854E-04,0,0,0 };
        //leg3
        const double rr_p1_iv[10]{ 2.3523814491,0,0,0,1.0480809884E-02,8.8825903772E-03,4.266754639E-03,0,0,0 };
        const double rr_p2_iv[10]{ 0.5868045476,0,0,0,6.9738871315E-03,6.8932585512E-03,5.6065308443E-04,0,0,0 };
        const double rr_p3_iv[10]{ 1.0111367876,0,0,0,1.1955933401E-02,1.1656650714E-02,5.8819775854E-04,0,0,0 };
        //leg4
        const double rf_p1_iv[10]{ 2.3523814491,0,0,0,1.0480809884E-02,8.8825903772E-03,4.266754639E-03,0,0,0 };
        const double rf_p2_iv[10]{ 0.5868045476,0,0,0,6.9738871315E-03,6.8932585512E-03,5.6065308443E-04,0,0,0 };
        const double rf_p3_iv[10]{ 1.0111367876,0,0,0,1.1955933401E-02,1.1656650714E-02,5.8819775854E-04,0,0,0 };



        // add part //
        auto& body = this->partPool().add<aris::dynamic::Part>("BODY", body_iv);
        //leg1
        auto& lf_p1 = this->partPool().add<aris::dynamic::Part>("LF_P1", lf_p1_iv);
        auto& lf_p2 = this->partPool().add<aris::dynamic::Part>("LF_P2", lf_p2_iv);
        auto& lf_p3 = this->partPool().add<aris::dynamic::Part>("LF_P3", lf_p3_iv);
        //leg2
        auto& lr_p1 = this->partPool().add<aris::dynamic::Part>("LR_P1", lr_p1_iv);
        auto& lr_p2 = this->partPool().add<aris::dynamic::Part>("LR_P2", lr_p2_iv);
        auto& lr_p3 = this->partPool().add<aris::dynamic::Part>("LR_P3", lr_p3_iv);
        //leg3
        auto& rr_p1 = this->partPool().add<aris::dynamic::Part>("RR_P1", rr_p1_iv);
        auto& rr_p2 = this->partPool().add<aris::dynamic::Part>("RR_P2", rr_p2_iv);
        auto& rr_p3 = this->partPool().add<aris::dynamic::Part>("RR_P3", rr_p3_iv);
        //leg4
        auto& rf_p1 = this->partPool().add<aris::dynamic::Part>("RF_P1", rf_p1_iv);
        auto& rf_p2 = this->partPool().add<aris::dynamic::Part>("RF_P2", rf_p2_iv);
        auto& rf_p3 = this->partPool().add<aris::dynamic::Part>("RF_P3", rf_p3_iv);

        // add geometry //
        this->ground().geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\ground.x_t");
        body.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\body.x_t");
        //leg1
        lf_p1.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\l11.x_t");
        lf_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\l12.x_t");
        lf_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\l13.x_t");
        //leg2
        lr_p1.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\l21.x_t");
        lr_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\l22.x_t");
        lr_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\l23.x_t");
        //leg3
        rr_p1.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\l31.x_t");
        rr_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\l32.x_t");
        rr_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\l33.x_t");
        //leg4
        rf_p1.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\l41.x_t");
        rf_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\l42.x_t");
        rf_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\11529\\Desktop\\ADAMS_model\\cpp_adams_dogv2_5\\l43.x_t");



        // add joints //
        //leg1
        auto& lf_r1 = this->addRevoluteJoint(lf_p1, body, leg_pe[0], std::array<double, 3>{1, 0, 0}.data());
        auto& lf_r2 = this->addRevoluteJoint(lf_p2, lf_p1, leg_pe[1], std::array<double, 3>{0, 0, 1}.data());
        auto& lf_r3 = this->addRevoluteJoint(lf_p3, lf_p2, leg_pe[2], std::array<double, 3>{0, 0, 1}.data());
        //leg2
        auto& lr_r1 = this->addRevoluteJoint(lr_p1, body, leg_pe[3], std::array<double, 3>{1, 0, 0}.data());
        auto& lr_r2 = this->addRevoluteJoint(lr_p2, lr_p1, leg_pe[4], std::array<double, 3>{0, 0, 1}.data());
        auto& lr_r3 = this->addRevoluteJoint(lr_p3, lr_p2, leg_pe[5], std::array<double, 3>{0, 0, 1}.data());
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
        auto body_ee_makj = this->ground().addMarker("body_ee_mak_j");

        auto& body_ee = this->generalMotionPool().add<aris::dynamic::GeneralMotion>("body_ee", &body_ee_maki, &body_ee_makj);
        auto& lf_ee = this->addPointMotion(lf_p3, this->ground(), ee_pos[0]);
        this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
        auto& lr_ee = this->addPointMotion(lr_p3, this->ground(), ee_pos[1]);
        this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
        auto& rr_ee = this->addPointMotion(rr_p3, this->ground(), ee_pos[2]);
        this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
        auto& rf_ee = this->addPointMotion(rf_p3, this->ground(), ee_pos[3]);
        this->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());


        // add force //
        auto& f1 = this->forcePool().add<aris::dynamic::GeneralForce>("f1", lf_ee.makI(), lf_ee.makJ());
        auto& f2 = this->forcePool().add<aris::dynamic::GeneralForce>("f2", lr_ee.makI(), lr_ee.makJ());
        auto& f3 = this->forcePool().add<aris::dynamic::GeneralForce>("f3", rr_ee.makI(), rr_ee.makJ());
        auto& f4 = this->forcePool().add<aris::dynamic::GeneralForce>("f4", rf_ee.makI(), rf_ee.makJ());
    

        auto& inverse_kinematic_solver = this->solverPool().add<aris::dynamic::InverseKinematicSolver>();
        auto& forward_kinematic_solver = this->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
        auto& inverse_dynamic_solver = this->solverPool().add<aris::dynamic::InverseDynamicSolver>();
        auto& forward_dynamic_solver = this->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

        auto& fix_body_universal = this->solverPool().add<aris::dynamic::UniversalSolver>();
        auto& stand_universal = this->solverPool().add<aris::dynamic::UniversalSolver>();
    
        auto& other = this->solverPool().add<aris::dynamic::UniversalSolver>();

        // 添加仿真器和仿真结果 //
        auto& adams = this->simulatorPool().add<aris::dynamic::AdamsSimulator>();
        auto& result = this->simResultPool().add<aris::dynamic::SimResult>();

        //    auto &force_slave = controller->slavePool().add<aris::control::EthercatSlave>();
        //    force_slave.scanInfoForCurrentSlave();
        //    force_slave.scanPdoForCurrentSlave();
        this->init();
    }
  
};
auto createModelQuadruped()->std::unique_ptr<aris::dynamic::Model>
{
    return std::unique_ptr<aris::dynamic::Model>(new Quad);
}
auto createControllerQuadruped()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);


#ifdef UNIX

    auto &force_slave = controller->slavePool().add<aris::control::EthercatSlave>();
    force_slave.scanInfoForCurrentSlave();
    force_slave.scanPdoForCurrentSlave();

#endif // WIN32



    for (aris::Size i = 0; i < 3; ++i)
    {
#ifdef ARIS_USE_ETHERCAT_SIMULATION
        double pos_offset[12]
        {            -0.0163824,
                     0.184725,
                     1.90999,
                     -0.193661,
                     0.289958,
                     0.609837,
                     -0.204495,
                     0.287226,
                     0.612374,
                     -0.184633,
                     0.708104,
                     -0.945939
            0,0,0,0,0,0 0
        };
#else
        double pos_offset[12]
        {

            0.461297,
            0.312223,
            0.228573,
            1.91165,
            -86.1291,
            -1.02921,
            -0.409439,
            -0.0275925,
            -9597.06,
            -0.126438,
            0.277727,
            0.122965



        };
#endif
        double pos_factor[12]
        {
            131072.0 *10 / 2 / PI,131072.0 *10 / 2 / PI,131072.0 * 20 / 2 / PI,
            16384.0 *20 / 2 / PI,16384.0 *20 / 2 / PI,16384.0 *40 / 2 / PI,
            16384.0 *20 / 2 / PI,16384.0 *20 / 2 / PI,16384.0 *40 / 2 / PI,
            131072.0 *10 / 2 / PI,131072.0 *10 / 2 / PI,131072.0 * 20 / 2 / PI

        };
        double max_pos[12]
        {
            PI,PI,PI,
            PI,PI,PI,
            PI,PI,PI,
            PI,PI,PI,
        };
        double min_pos[12]
        {
            -PI,-PI,-PI,
            -PI,-PI,-PI,
            -PI,-PI,-PI,
            -PI,-PI,-PI,
        };
        double max_vel[12]
        {
            350 / 60 *    PI, 350 / 60 *    PI, 350 / 60 *    PI,
            350 / 60 *    PI, 350 / 60 *    PI, 350 / 60 *    PI,
            350 / 60 *    PI, 350 / 60 *    PI, 350 / 60 *    PI,
            350 / 60 *    PI, 350 / 60 *    PI, 350 / 60 *    PI


        };
        double max_acc[12]
        {
            30000,  30000,  30000,
            30000,  30000,  30000,
            30000,  30000,  30000,
            30000,  30000,  30000,
        };
        //int phy_id[12]={1,2,3,6,5,4,7,8,9,10,11,12};
        int phy_id[12]={3,2,1,4,5,6,9,8,7,10,11,12};

        std::string xml_str =
            "<EthercatMotor phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
            " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
            " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
            " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"10.0\" max_vel_following_error=\"20.0\""
            " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
            "	<SyncManagerPoolObject>"
            "		<SyncManager is_tx=\"false\"/>"
            "		<SyncManager is_tx=\"true\"/>"
            "		<SyncManager is_tx=\"false\">"
            "			<Pdo index=\"0x1605\" is_tx=\"false\">"
            "				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"max_toq\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "		<SyncManager is_tx=\"true\">"
            "			<Pdo index=\"0x1A07\" is_tx=\"true\">"
            "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
            "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "	</SyncManagerPoolObject>"
            "</EthercatMotor>";

        auto& s = controller->slavePool().add<aris::control::EthercatMotor>();
        aris::core::fromXmlString(s, xml_str);



#ifdef WIN32
        dynamic_cast<aris::control::E	cs.interfacePool().add<aris::server::ProgramWebInterface>("", "5866", aris::core::Socket::WEB);
        cs.interfacePool().add<aris::server::HttpInterface>("", "8001", "D:/UI/www");
thercatMotor&>(controller->slavePool().back()).setVirtual(true);
#endif

#ifndef WIN32
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
#endif

    };
    return controller;
}
auto createPlanQuadruped()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);
    
    plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Home>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Show>();
    plan_root->planPool().add<aris::plan::Sleep>();
    plan_root->planPool().add<aris::plan::Clear>();
    plan_root->planPool().add<aris::plan::Recover>();
    auto &rs = plan_root->planPool().add<aris::plan::Reset>();
    rs.command().findParam("pos")->setDefaultValue("{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}");

    auto &mvaj = plan_root->planPool().add<aris::plan::MoveAbsJ>();
    mvaj.command().findParam("vel")->setDefaultValue("0.1");

    plan_root->planPool().add<aris::plan::MoveL>();
    plan_root->planPool().add<aris::plan::MoveJ>();

    plan_root->planPool().add<aris::plan::GetXml>();
    plan_root->planPool().add<aris::plan::SetXml>();
    plan_root->planPool().add<aris::plan::Start>();
    plan_root->planPool().add<aris::plan::Stop>();

    //--------------自己写的命令-------------------//
    plan_root->planPool().add<SetMaxToq>();
    plan_root->planPool().add<DogReadJoint>();
    plan_root->planPool().add<DogReadForce>();
    plan_root->planPool().add<DogInitPos>();
    plan_root->planPool().add<DogSetWalkMode>();
    plan_root->planPool().add<DogMoveJoint>();
  //  plan_root->planPool().add<DogTorqueControl>();
    plan_root->planPool().add<DogHome>();
  ////  plan_root->planPool().add<DogSwitchPrePose>();
    plan_root->planPool().add<DogPrepare>();
    plan_root->planPool().add<DogUpDown>();
    plan_root->planPool().add<DogTaBu>();
    plan_root->planPool().add<DogForward>();
    plan_root->planPool().add<DogBack>();
    plan_root->planPool().add<DogLeft>();
    plan_root->planPool().add<DogRight>();
    plan_root->planPool().add<DogTurn>();
    plan_root->planPool().add<DogPitch>();
    plan_root->planPool().add<DogRoll>();
    plan_root->planPool().add<DogYaw>();
    plan_root->planPool().add<DogPose>();
  //  plan_root->planPool().add<TestFilter>();

    plan_root->planPool().add<DogDynamicTest>();

    plan_root->planPool().add<DogRun>();
#ifdef UNIX
    plan_root->planPool().add<ControlThread>();
#endif // UNIX

    
    return plan_root;
}



}
