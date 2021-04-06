﻿#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>

#include "robot.h"
#include"plan.h"
#include"kinematics.h"

double input_angle[12] = {0};
double init_pos_angle[12] = { 0 };
std::string gait="trot";
std::string prepose="same";
//输出参数，模型曲线测试使用
double file_current_leg[12] = { 0 };
double file_current_body[16] = { 0 };
double time_test = 0;

double body_cm[9]={0.1,0,-0.05,0.1,0,0.1,0,0,-0.05};

using namespace aris::dynamic;
using namespace aris::plan;
//const double PI = aris::PI;
namespace robot
{
//设置力矩
auto SetMaxToq::prepareNrt()->void
{
    for(auto &m:motorOptions()) m = aris::plan::Plan::CHECK_NONE;
}
auto SetMaxToq::executeRT()->int
{
    auto &cout = controller()->mout();

    for (int i = 0; i < 12; ++i)
    {
        std::uint16_t max_toq=1000;
        this->ecController()->motionPool()[i].writePdo(0x6072,0,max_toq);
    }
    //返回0表示正常结束，返回负数表示报错，返回正数表示正在执行
    return 1000 - count();
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

    TCurve s1(1, 4);
    s1.getCurveParam();
    EllipseTrajectory e1(0.4, 0.10, 0, s1);

    //步态规划
    if (gait == "trot" && prepose == "same")//trot & same
    {
        trotPlanSameLeg(1, count()-1, &e1,init_pos_angle );
    }
    else if (gait == "walk" && prepose == "same") //walk & same
    {
        walkPlanSameLeg(1, count()-1, &e1, init_pos_angle);
    }
    else if (gait == "walk" && prepose == "symmetry")  //walk & symmetry
    {
        walkPlanSymmetryLeg(1, count()-1, &e1, init_pos_angle);
    }
    else //
    {
        mout() << "input dog_error" << std::endl;
    }

    for (int i = 0; i < 12; ++i)
    {
        std::cout << init_pos_angle[i] << std::endl;
    }

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
        begin_angle[0] = controller()->motionPool()[0].targetPos();
//        begin_angle[1] = controller()->motionPool()[1].targetPos();
//        begin_angle[2] = controller()->motionPool()[2].targetPos();
//        begin_angle[3] = controller()->motionPool()[3].targetPos();
//        begin_angle[4] = controller()->motionPool()[4].targetPos();
//        begin_angle[5] = controller()->motionPool()[5].targetPos();
//        begin_angle[6] = controller()->motionPool()[6].targetPos();
//        begin_angle[7] = controller()->motionPool()[7].targetPos();
//        begin_angle[8] = controller()->motionPool()[8].targetPos();
//        begin_angle[9] = controller()->motionPool()[9].targetPos();
//        begin_angle[10] = controller()->motionPool()[10].targetPos();
//        begin_angle[11] = controller()->motionPool()[11].targetPos();
    }
        TCurve s1(1,3);
        s1.getCurveParam();

    double angle0 = begin_angle[0] + dir_ * PI * s1.getTCurve(count());
//    double angle1 = begin_angle[1] + dir_ * PI * s1.getTCurve(count());
//    double angle2 = begin_angle[2] + dir_ * PI * s1.getTCurve(count());
//    double angle3 = begin_angle[3] + dir_ * PI * s1.getTCurve(count());
//    double angle4 = begin_angle[4] + dir_ * PI * s1.getTCurve(count());
//    double angle5 = begin_angle[5] + dir_ * PI * s1.getTCurve(count());
//    double angle6 = begin_angle[6] + dir_ * PI * s1.getTCurve(count());
//    double angle7 = begin_angle[7] + dir_ * PI * s1.getTCurve(count());
//    double angle8 = begin_angle[8] + dir_ * PI * s1.getTCurve(count());
//    double angle9 = begin_angle[9] + dir_ * PI * s1.getTCurve(count());
//    double angle10 = begin_angle[10] + dir_ * PI * s1.getTCurve(count());
//    double angle11 = begin_angle[11] + dir_ * PI * s1.getTCurve(count());



    controller()->motionPool()[0].setTargetPos(angle0);
//    controller()->motionPool()[1].setTargetPos(angle1);
//    controller()->motionPool()[2].setTargetPos(angle2);
//    controller()->motionPool()[3].setTargetPos(angle3);
//    controller()->motionPool()[4].setTargetPos(angle4);
//    controller()->motionPool()[5].setTargetPos(angle5);
//    controller()->motionPool()[6].setTargetPos(angle6);
//    controller()->motionPool()[7].setTargetPos(angle7);
//    controller()->motionPool()[8].setTargetPos(angle8);
//    controller()->motionPool()[6].setTargetPos(angle9);
//    controller()->motionPool()[7].setTargetPos(angle10);
//    controller()->motionPool()[8].setTargetPos(angle11);
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
    if (count() == 1)this->master()->logFileRawName("home");
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
        begin_angle[10] = controller()->motionPool()[10].targetPos();
        begin_angle[11] = controller()->motionPool()[11].targetPos();
    }

    TCurve s1(5,2);//0.9s
    s1.getCurveParam();

    for(int i=0;i<12;++i)
    {
        angle[i] = begin_angle[i] + (0-begin_angle[i]) * s1.getTCurve(count());
    }

    for(int i=0;i<12;++i)
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

//行走步态和曲腿模式设置
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


//轴空间初始位姿切换
auto DogSwitchPrePose::prepareNrt()->void
{
    prepose_ = cmdParams().at("prepose");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogSwitchPrePose::executeRT()->int
{
    static double begin_angle[12] = { 0 };
    double angle[12] = { 0 };
  
    double distance_same[12] = { 0 };
    double distance_symmetry[12] = { 0 };

    if (prepose_ == "same")
    {
        for (int i = 0; i < 12; ++i)
        {
            distance_same[i] = init_pos_angle[i];
            distance_symmetry[i] = init_pos_angle[i];
        }
    }
    else if (prepose_ == "symmetry")//symmetry
    {
        distance_symmetry[4] = -distance_symmetry[4]; //前后腿对称，14相同，23相同
        distance_symmetry[5] = -distance_symmetry[5];
        distance_symmetry[7] = -distance_symmetry[7];
        distance_symmetry[8] = -distance_symmetry[8];
    }
    else
    {
        mout() << "input error" << std::endl;
    }

    if (count() == 1)
    {
        this->master()->logFileRawName("switchpose");
        begin_angle[0] = controller()->motionPool()[0].actualPos();
        begin_angle[1] = controller()->motionPool()[1].actualPos();
        begin_angle[2] = controller()->motionPool()[2].actualPos()/2;
        begin_angle[3] = controller()->motionPool()[3].actualPos();
        begin_angle[4] = controller()->motionPool()[4].actualPos();
        begin_angle[5] = controller()->motionPool()[5].actualPos()/2;
        begin_angle[6] = controller()->motionPool()[6].actualPos();
        begin_angle[7] = controller()->motionPool()[7].actualPos();
        begin_angle[8] = controller()->motionPool()[8].actualPos()/2;
        begin_angle[9] = controller()->motionPool()[9].actualPos();
        begin_angle[10] = controller()->motionPool()[10].actualPos();
        begin_angle[11] = controller()->motionPool()[11].actualPos()/2;
    }


    TCurve s1(5, 2);//0.9s
    s1.getCurveParam();

    if (prepose_ == "same")//same
    {
        for (int i = 0; i < 12; ++i)
        {
            angle[i] = begin_angle[i] + (distance_same[i] - begin_angle[i]) * s1.getTCurve(count());
        }
    }
    else if(prepose_=="symmetry")//symmetry
    {
        for (int i = 0; i < 12; ++i)
        {
            angle[i] = begin_angle[i] + (distance_symmetry[i] - begin_angle[i]) * s1.getTCurve(count());
        }
    }
    else
    {
        mout() << "input error" << std::endl;
    }

    prepose = prepose_;

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; ++i)
        {
            lout() << angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";
        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << file_current_leg[i] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(2 * angle[i]);
        else
            controller()->motionPool()[i].setTargetPos(angle[i]);
    }
    return s1.getTc() * 1000 - count();
}
DogSwitchPrePose::DogSwitchPrePose(const std::string& name) 
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_switchpose\">"
        "<Param name=\"prepose\" default=\"same\" abbreviation=\"p\"/>"
        "</Command>");
}
DogSwitchPrePose::~DogSwitchPrePose() = default;


//轴空间准备
auto DogPrepare::prepareNrt()->void
{
    for(auto &m:motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogPrepare::executeRT()->int
{
    static double begin_angle[12]={0};
    double angle[12]={0};

    if (count() == 1)this->master()->logFileRawName("prepare");

    double distance[12] = { 0 };
 
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
    }

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; ++i)
        {
            lout() << angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";
        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << file_current_leg[i] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(2 * angle[i]);
        else
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

    if (count() == 1)
    {
        input_angle[0] = controller()->motionPool()[0].actualPos();
        input_angle[1] = controller()->motionPool()[1].actualPos();
        input_angle[2] = controller()->motionPool()[2].actualPos();
        input_angle[3] = controller()->motionPool()[3].actualPos();
        input_angle[4] = controller()->motionPool()[4].actualPos();
        input_angle[5] = controller()->motionPool()[5].actualPos();
        input_angle[6] = controller()->motionPool()[6].actualPos();
        input_angle[7] = controller()->motionPool()[7].actualPos();
        input_angle[8] = controller()->motionPool()[8].actualPos();
        input_angle[9] = controller()->motionPool()[9].actualPos();
        input_angle[10] = controller()->motionPool()[10].actualPos();
        input_angle[11] = controller()->motionPool()[11].actualPos();
    }
    TCurve s1(5, 2);//0.9s
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);

    
    // test read toeque //
    int16_t torque[12]={0};
    for(int i=0;i<12;++i)
    {
        this->ecController()->motionPool()[i].readPdo(0x6077,0x00,torque[i]);
        lout()<< torque[i] << "\t";
    }
    lout() << std::endl;

    //步态规划
    if (prepose == "same")// same
    {
        ret = updownPlanSameLeg(count() - 1, &e1, distance_, input_angle);
    }
    else if (prepose == "symmetry")  //symmetry
    {
        ret = updownPlanSymmetry(count() - 1, &e1, distance_, input_angle);
    }
    else //
    {
        mout() << "input error" << std::endl;
    }


//    //输出角度，用于仿真测试
//    {
//        for (int i = 0; i < 12; ++i)
//        {
//            lout() << input_angle[i] << "\t";
//        }
//        time_test += 0.001;
//        lout() << time_test << "\t";

//        //输出身体和足尖曲线
//        for (int i = 0; i < 12; ++i)
//        {
//            lout() << file_current_leg[i] << "\t";
//        }
//        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
//    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(2 * input_angle[i]);
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
    if (count() == 1)
    {
        input_angle[0] = controller()->motionPool()[0].actualPos();
        input_angle[1] = controller()->motionPool()[1].actualPos();
        input_angle[2] = controller()->motionPool()[2].actualPos();
        input_angle[3] = controller()->motionPool()[3].actualPos();
        input_angle[4] = controller()->motionPool()[4].actualPos();
        input_angle[5] = controller()->motionPool()[5].actualPos();
        input_angle[6] = controller()->motionPool()[6].actualPos();
        input_angle[7] = controller()->motionPool()[7].actualPos();
        input_angle[8] = controller()->motionPool()[8].actualPos();
        input_angle[9] = controller()->motionPool()[9].actualPos();
        input_angle[10] = controller()->motionPool()[10].actualPos();
        input_angle[11] = controller()->motionPool()[11].actualPos();
    }

    TCurve s1(1, 4);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0.1, 0, s1);

    int ret = 1;
    //步态规划
    if ( prepose == "same")// same
    {
        ret = trotPlanSameLeg(step_, count() - 1, &e1, input_angle);
    }
    else if (prepose == "symmetry")  //symmetry
    {
        ret = trotPlanSymmetryLeg(step_, count() - 1, &e1, input_angle);
    }
    else //
    {
        mout() << "input error" << std::endl;
    }

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << file_current_leg[i] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(2 * input_angle[i]);
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

    if (count() == 1)
    {
        input_angle[0] = controller()->motionPool()[0].actualPos();
        input_angle[1] = controller()->motionPool()[1].actualPos();
        input_angle[2] = controller()->motionPool()[2].actualPos();
        input_angle[3] = controller()->motionPool()[3].actualPos();
        input_angle[4] = controller()->motionPool()[4].actualPos();
        input_angle[5] = controller()->motionPool()[5].actualPos();
        input_angle[6] = controller()->motionPool()[6].actualPos();
        input_angle[7] = controller()->motionPool()[7].actualPos();
        input_angle[8] = controller()->motionPool()[8].actualPos();
        input_angle[9] = controller()->motionPool()[9].actualPos();
        input_angle[10] = controller()->motionPool()[10].actualPos();
        input_angle[11] = controller()->motionPool()[11].actualPos();
    }

    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0.2, 0.05, 0, s1);


    //步态规划
    if (gait == "trot" && prepose == "same")//trot & same
    {
        ret = trotPlanSameLeg(step_, count() - 1, &e1, input_angle);
    }
    else if (gait =="walk"  && prepose == "same") //walk & same
    {
        //ret = walkPlanSameLeg(step_, count() - 1, &e1, input_angle);
        //ret = walkPlanSameLeg2(step_, count() - 1, &e1, input_angle,z_);
        ret = walkPlanSameLeg3(step_, count() - 1, &e1, &s1,input_angle,body_cm);
    }
    else if (gait == "walk" && prepose == "symmetry")  //walk & symmetry
    {
        ret = walkPlanSymmetryLeg(step_, count() - 1, &e1, input_angle);
    }
    else //
    {
        mout() << "input error" << std::endl;
    }

//    //输出角度，用于仿真测试
    double data_angle [12]={0};
    for(int i=0 ;i<12;++i)
    {
        data_angle[i] = controller()->motionPool()[i].actualPos();
        lout() << data_angle[i] << "\t";
    }
    lout() << std::endl;



//        //输出电机角度
//        for (int i = 0; i < 12; ++i)
//        {
//            lout() << data_angle[i] << "\t";
//        }
//        time_test += 0.001;

//        //输出身体和足尖曲线
//        for (int i = 0; i < 12; ++i)
//        {
//            lout() << file_current_leg[i] << "\t";
//        }
//        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
//    }


    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(2 * input_angle[i]);
        else
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
    if (count() == 1)
    {
        input_angle[0] = controller()->motionPool()[0].actualPos();
        input_angle[1] = controller()->motionPool()[1].actualPos();
        input_angle[2] = controller()->motionPool()[2].actualPos();
        input_angle[3] = controller()->motionPool()[3].actualPos();
        input_angle[4] = controller()->motionPool()[4].actualPos();
        input_angle[5] = controller()->motionPool()[5].actualPos();
        input_angle[6] = controller()->motionPool()[6].actualPos();
        input_angle[7] = controller()->motionPool()[7].actualPos();
        input_angle[8] = controller()->motionPool()[8].actualPos();
        input_angle[9] = controller()->motionPool()[9].actualPos();
        input_angle[10] = controller()->motionPool()[10].actualPos();
        input_angle[11] = controller()->motionPool()[11].actualPos();
    }

    TCurve s1(1, 4);
    s1.getCurveParam();
    EllipseTrajectory e1(-0.15, 0.08, 0, s1);

    //步态规划
    if (gait == "trot" && prepose == "same")//trot & same
    {
        ret = trotPlanSameLeg(step_, count() - 1, &e1, input_angle);
    }
    else if (gait == "walk" && prepose == "same") //walk & same
    {
        ret = walkPlanSameLeg(step_, count() - 1, &e1, input_angle);
    }
    else if (gait == "walk" && prepose == "symmetry")  //walk & symmetry
    {
        ret = walkPlanSymmetryLeg(step_, count() - 1, &e1, input_angle);
    }
    else //
    {
        mout() << "input error" << std::endl;
    }

    //输出角度，用于仿真测试
    {
        //输出电机角度
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << file_current_leg[i] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(2 * input_angle[i]);
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
    if (count() == 1)
    {
        this->master()->logFileRawName("left");
        input_angle[0] = controller()->motionPool()[0].actualPos();
        input_angle[1] = controller()->motionPool()[1].actualPos();
        input_angle[2] = controller()->motionPool()[2].actualPos();
        input_angle[3] = controller()->motionPool()[3].actualPos();
        input_angle[4] = controller()->motionPool()[4].actualPos();
        input_angle[5] = controller()->motionPool()[5].actualPos();
        input_angle[6] = controller()->motionPool()[6].actualPos();
        input_angle[7] = controller()->motionPool()[7].actualPos();
        input_angle[8] = controller()->motionPool()[8].actualPos();
        input_angle[9] = controller()->motionPool()[9].actualPos();
        input_angle[10] = controller()->motionPool()[10].actualPos();
        input_angle[11] = controller()->motionPool()[11].actualPos();
    }
    //轨迹规划
    TCurve s1(1, 4);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0.08, -0.1, s1);

    //步态规划
    if (gait == "trot" && prepose == "same")//trot & same
    {
        ret = trotPlanSameLeg(step_, count() - 1, &e1, input_angle);
    }
    else if (gait == "walk" && prepose == "same") //walk & same
    {
        ret = walkPlanSameLeg(step_, count() - 1, &e1, input_angle);
    }
    else if (gait == "walk" && prepose == "symmetry")  //walk & symmetry
    {
        ret = walkPlanSymmetryLeg(step_, count() - 1, &e1, input_angle);
    }
    else //
    {
        mout() << "input error" << std::endl;
    }

    //输出角度，用于仿真测试
    {
        //输出电机角度
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << file_current_leg[i] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(2 * input_angle[i]);
        else
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
    if (count() == 1)
    {
        this->master()->logFileRawName("right");
        input_angle[0] = controller()->motionPool()[0].actualPos();
        input_angle[1] = controller()->motionPool()[1].actualPos();
        input_angle[2] = controller()->motionPool()[2].actualPos();
        input_angle[3] = controller()->motionPool()[3].actualPos();
        input_angle[4] = controller()->motionPool()[4].actualPos();
        input_angle[5] = controller()->motionPool()[5].actualPos();
        input_angle[6] = controller()->motionPool()[6].actualPos();
        input_angle[7] = controller()->motionPool()[7].actualPos();
        input_angle[8] = controller()->motionPool()[8].actualPos();
        input_angle[9] = controller()->motionPool()[9].actualPos();
        input_angle[10] = controller()->motionPool()[10].actualPos();
        input_angle[11] = controller()->motionPool()[11].actualPos();
    }

    TCurve s1(1, 4);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0.08,0.1, s1);

    //步态规划
    if (gait == "trot" && prepose == "same")//trot & same
    {
        ret = trotPlanSameLeg(step_, count() - 1, &e1, input_angle);
    }
    else if (gait == "walk" && prepose == "same") //walk & same
    {
        ret = walkPlanSameLeg(step_, count() - 1, &e1, input_angle);
    }
    else if (gait == "walk" && prepose == "symmetry")  //walk & symmetry
    {
        ret = walkPlanSymmetryLeg(step_, count() - 1, &e1, input_angle);
    }
    else //
    {
        mout() << "input error" << std::endl;
    }

    //输出角度，用于仿真测试
    {
        //输出电机角度
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << file_current_leg[i] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(2 * input_angle[i]);
        else
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

//pitch
auto DogPitch::prepareNrt()->void
{
    turn_angle_ = doubleParam("angle");
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogPitch::executeRT()->int
{
    if (count() == 1)this->master()->logFileRawName("pitch");
    int ret = 0;
    if (count() == 1)
    {
        input_angle[0] = controller()->motionPool()[0].actualPos();
        input_angle[1] = controller()->motionPool()[1].actualPos();
        input_angle[2] = controller()->motionPool()[2].actualPos();
        input_angle[3] = controller()->motionPool()[3].actualPos();
        input_angle[4] = controller()->motionPool()[4].actualPos();
        input_angle[5] = controller()->motionPool()[5].actualPos();
        input_angle[6] = controller()->motionPool()[6].actualPos();
        input_angle[7] = controller()->motionPool()[7].actualPos();
        input_angle[8] = controller()->motionPool()[8].actualPos();
        input_angle[9] = controller()->motionPool()[9].actualPos();
        input_angle[10] = controller()->motionPool()[10].actualPos();
        input_angle[11] = controller()->motionPool()[11].actualPos();
    }

    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);

    if (std::abs(turn_angle_) > 10)
    {
        mout() << "input exceed mas value" << std::endl;
    }
    else
    {
        BodyPose body_pose(0, 0, turn_angle_, s1);
        ret = posePlan(count() - 1, &e1, &body_pose, input_angle);
    }

    //输出角度，用于仿真测试
    {
        //输出电机角度
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << file_current_leg[i] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(2 * input_angle[i]);
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
    if (count() == 1)this->master()->logFileRawName("roll");
    int ret = 0;
    if (count() == 1)
    {
        input_angle[0] = controller()->motionPool()[0].actualPos();
        input_angle[1] = controller()->motionPool()[1].actualPos();
        input_angle[2] = controller()->motionPool()[2].actualPos();
        input_angle[3] = controller()->motionPool()[3].actualPos();
        input_angle[4] = controller()->motionPool()[4].actualPos();
        input_angle[5] = controller()->motionPool()[5].actualPos();
        input_angle[6] = controller()->motionPool()[6].actualPos();
        input_angle[7] = controller()->motionPool()[7].actualPos();
        input_angle[8] = controller()->motionPool()[8].actualPos();
        input_angle[9] = controller()->motionPool()[9].actualPos();
        input_angle[10] = controller()->motionPool()[10].actualPos();
        input_angle[11] = controller()->motionPool()[11].actualPos();
    }

    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);
    if (std::abs(turn_angle_) > 20)
    {
        mout() << "input exceed mas value" << std::endl;
    }
    else
    {
        BodyPose body_pose(turn_angle_, 0, 0, s1);
        ret = posePlan(count() - 1, &e1, &body_pose, input_angle);
    }
    
   
    //输出角度，用于仿真测试
    {
        //输出电机角度
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << file_current_leg[i] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(2 * input_angle[i]);
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
    if (count() == 1)this->master()->logFileRawName("yaw");
    int ret = 0;
    if (count() == 1)
    {
        input_angle[0] = controller()->motionPool()[0].actualPos();
        input_angle[1] = controller()->motionPool()[1].actualPos();
        input_angle[2] = controller()->motionPool()[2].actualPos();
        input_angle[3] = controller()->motionPool()[3].actualPos();
        input_angle[4] = controller()->motionPool()[4].actualPos();
        input_angle[5] = controller()->motionPool()[5].actualPos();
        input_angle[6] = controller()->motionPool()[6].actualPos();
        input_angle[7] = controller()->motionPool()[7].actualPos();
        input_angle[8] = controller()->motionPool()[8].actualPos();
        input_angle[9] = controller()->motionPool()[9].actualPos();
        input_angle[10] = controller()->motionPool()[10].actualPos();
        input_angle[11] = controller()->motionPool()[11].actualPos();
    }

    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);
    BodyPose body_pose(0, turn_angle_, 0, s1);
    ret = posePlan(count()-1, &e1, &body_pose, input_angle);

    //输出角度，用于仿真测试
    {
        //输出电机角度
        for (int i = 0; i < 12; ++i)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int i = 0; i < 12; ++i)
        {
            lout() << file_current_leg[i] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; ++i)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(2 * input_angle[i]);
        else
            controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    return ret;
}
DogYaw::DogYaw(const std::string& name) 
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_yaw\">"
        "	<Param name=\"angle\" default=\"10\" abbreviation=\"d\"/>"
        "</Command>");
}
DogYaw::~DogYaw() = default;



// 单关节正弦往复轨迹 //
struct MoveJSParam
{
    double j1;
    double time;
    uint32_t timenum;
};
auto MoveJS::prepareNrt()->void
{
    MoveJSParam param;

    param.j1 = 0.0;
    param.time = 0.0;
    param.timenum = 0;

    for (auto& p : cmdParams())
    {
        if (p.first == "j1")
        {
            if (p.second == "current_pos")
            {
                param.j1 = controller()->motionPool()[0].actualPos();
            }
            else
            {
                param.j1 = doubleParam(p.first);
            }

        }
        else if (p.first == "time")
        {
            param.time = doubleParam(p.first);
        }
        else if (p.first == "timenum")
        {
            param.timenum = int32Param(p.first);
        }
    }
    this->param() = param;
    std::vector<std::pair<std::string, std::any>> ret_value;
    for (auto& option : motorOptions())	option |= NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | NOT_CHECK_POS_CONTINUOUS;
    ret() = ret_value;
}
auto MoveJS::executeRT()->int
{
    auto& param = std::any_cast<MoveJSParam&>(this->param());
    auto time = static_cast<int32_t>(param.time * 1000);
    auto totaltime = static_cast<int32_t>(param.timenum * time);
    static double begin_pjs;
    static double step_pjs;
    // 访问主站 //
    auto& cout = controller()->mout();

    if ((1 <= count()) && (count() <= time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
        }
        step_pjs = begin_pjs + param.j1 * (1 - std::cos(2 * PI * count() / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }
    else if ((time / 2 < count()) && (count() <= totaltime - time / 2))
    {
        // 获取当前起始点位置 //
        if (count() == time / 2 + 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
        }

        step_pjs = begin_pjs - 2 * param.j1 * (1 - std::cos(2 * PI * (count() - time / 2) / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }
    else if ((totaltime - time / 2 < count()) && (count() <= totaltime))
    {
        // 获取当前起始点位置 //
        if (count() == totaltime - time / 2 + 1)
        {
            begin_pjs = controller()->motionPool()[0].actualPos();
            step_pjs = controller()->motionPool()[0].actualPos();
        }
        step_pjs = begin_pjs - param.j1 * (1 - std::cos(2 * PI * (count() - totaltime + time / 2) / time)) / 2;
        controller()->motionPool().at(0).setTargetPos(step_pjs);
    }

    // 打印 //
    if (count() % 100 == 0)
    {
        cout << "pos" << ":" << controller()->motionAtAbs(0).actualPos() << "  ";
        cout << std::endl;
    }

    // log //
    auto& lout = controller()->lout();
    lout << controller()->motionAtAbs(0).targetPos() << ",";
    lout << std::endl;

    return totaltime - count();
}
auto MoveJS::collectNrt()->void {}
MoveJS::MoveJS(const std::string& name)
{
    aris::core::fromXmlString(command(),
        "<Command name=\"moveJS\">"
        "	<GroupParam>"
        "		<Param name=\"j1\" default=\"current_pos\"/>"
        "		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
        "		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
        "	</GroupParam>"
        "</Command>");
}



//auto createModelQuadruped()->std::unique_ptr<aris::dynamic::Model>
//{
//    std::unique_ptr<aris::dynamic::Model> quad = std::make_unique<aris::dynamic::Model>();


//    // set gravity //
//    const double gravity[6]{ 0.0,-9.8,0.0,0.0,0.0,0.0 };

//    quad->environment().setGravity(gravity);

//    //define joint pos //
//    const double leg_pe[12][3]{
//        {  kBodyLong / 2,   0, -kBodyWidth / 2      },
//        {  kBodyLong / 2,   0, -kBodyWidth / 2 - L1 },
//        {  kBodyLong / 2, -L2, -kBodyWidth / 2 - L1 },
//        { -kBodyLong / 2,   0, -kBodyWidth / 2      },
//        { -kBodyLong / 2,   0, -kBodyWidth / 2 - L1 },
//        { -kBodyLong / 2, -L2, -kBodyWidth / 2 - L1 },
//        { -kBodyLong / 2,   0,  kBodyWidth / 2      },
//        { -kBodyLong / 2,   0,  kBodyWidth / 2 + L1 },
//        { -kBodyLong / 2, -L2,  kBodyWidth / 2 + L1 },
//        {  kBodyLong / 2,   0,  kBodyWidth / 2      },
//        {  kBodyLong / 2,   0,  kBodyWidth / 2 + L1 },
//        {  kBodyLong / 2, -L2,  kBodyWidth / 2 + L1 },
//    };
//    //define ee pos //
//    const double ee_pos[4][6]{
//    { kBodyLong / 2, -L3 - L2, -(kBodyWidth / 2) - L1,0.0, 0.0, 0.0},   //leg1 ->012
//    {-kBodyLong / 2, -L3 - L2, -(kBodyWidth / 2) - L1,0.0, 0.0, 0.0},   //leg2 ->345
//    {-kBodyLong / 2, -L3 - L2,  (kBodyWidth / 2) + L1,0.0, 0.0, 0.0},   //leg3 ->678
//    { kBodyLong / 2, -L3 - L2,  (kBodyWidth / 2) + L1,0.0, 0.0, 0.0},   //leg4 ->91011
//    };
    
//    //iv:  10x1 惯量矩阵向量[m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
//    // define iv param //  材料都为铝 总重25.037kg
//    const double body_iv[10]{ 11.244602964,0,0,0,0.6510608871,0.5989223304,7.9786361064E-02,0,0,0 };
//    //leg1
//    const double lf_p1_iv[10]{ 2.1374710038,0,0,0,5.5931718868E-03,5.472611078E-03,2.6340502655E-03,0,0,0 };
//    const double lf_p2_iv[10]{ 0.4968154968,0,0,0,4.3644561758E-03,4.270698531E-03,2.3175168139E-04,0,0,0 };
//    const double lf_p3_iv[10]{ 0.8145237429,0,0,0,1.0780253535E-02,1.076892825E-02,03.695341839E-04,0,0,0 };
//    //leg2
//    const double lr_p1_iv[10]{ 2.1374710038,0,0,0,5.5931718868E-03,5.472611078E-03,2.6340502655E-03,0,0,0 };
//    const double lr_p2_iv[10]{ 0.4968154968,0,0,0,4.3644561758E-03,4.270698531E-03,2.3175168139E-04,0,0,0 };
//    const double lr_p3_iv[10]{ 0.8145237429,0,0,0,1.0780253535E-02,1.076892825E-02,03.695341839E-04,0,0,0 };
//    //leg3
//    const double rr_p1_iv[10]{ 2.1374710037,0,0,0,5.5931718868E-03,5.4726110785E-03,2.6340502654E-03,0,0,0 };
//    const double rr_p2_iv[10]{ 0.4968154968,0,0,0,4.3644561728E-03,4.2706985298E-03,2.3175167966E-04,0,0,0 };
//    const double rr_p3_iv[10]{ 0.814523742,0,0,0,1.0780253649E-02,1.0768928365E-02,3.6953418362E-04,0,0,0 };
//    //leg4
//    const double rf_p1_iv[10]{ 2.1374710038,0,0,0,5.5931718868E-03,5.472611078E-03,2.6340502655E-03,0,0,0 };
//    const double rf_p2_iv[10]{ 0.4968154968,0,0,0,4.3644561758E-03,4.270698531E-03,2.3175168139E-04,0,0,0 };
//    const double rf_p3_iv[10]{ 0.8145237429,0,0,0,1.0780253535E-02,1.076892825E-02,03.695341839E-04,0,0,0 };

//    // add part //
//    auto& body = quad->partPool().add<aris::dynamic::Part>("BODY", body_iv);
//    //leg1
//    auto& lf_p1 = quad->partPool().add<aris::dynamic::Part>("LF_P1", lf_p1_iv);
//    auto& lf_p2 = quad->partPool().add<aris::dynamic::Part>("LF_P2", lf_p2_iv);
//    auto& lf_p3 = quad->partPool().add<aris::dynamic::Part>("LF_P3", lf_p3_iv);
//    //leg2
//    auto& lr_p1 = quad->partPool().add<aris::dynamic::Part>("LR_P1", lr_p1_iv);
//    auto& lr_p2 = quad->partPool().add<aris::dynamic::Part>("LR_P2", lr_p2_iv);
//    auto& lr_p3 = quad->partPool().add<aris::dynamic::Part>("LR_P3", lr_p3_iv);
//    //leg3
//    auto& rr_p1 = quad->partPool().add<aris::dynamic::Part>("RR_P1", rr_p1_iv);
//    auto& rr_p2 = quad->partPool().add<aris::dynamic::Part>("RR_P2", rr_p2_iv);
//    auto& rr_p3 = quad->partPool().add<aris::dynamic::Part>("RR_P3", rr_p3_iv);
//    //leg4
//    auto& rf_p1 = quad->partPool().add<aris::dynamic::Part>("RF_P1", rf_p1_iv);
//    auto& rf_p2 = quad->partPool().add<aris::dynamic::Part>("RF_P2", rf_p2_iv);
//    auto& rf_p3 = quad->partPool().add<aris::dynamic::Part>("RF_P3", rf_p3_iv);

//    // add geometry //
//    quad->ground().geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\ground.x_t");
//    body.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\body.x_t");
//    //leg1
//    lf_p1.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l11.x_t");
//    lf_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l12.x_t");
//    lf_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l13.x_t");
//    //leg2
//    lr_p1.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l21.x_t");
//    lr_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l22.x_t");
//    lr_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l23.x_t");
//    //leg3
//    rr_p1.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l31.x_t");
//    rr_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l32.x_t");
//    rr_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l33.x_t");
//    //leg4
//    rf_p1.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l41.x_t");
//    rf_p2.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l42.x_t");
//    rf_p3.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\DELL1\\Desktop\\ADAMS_model\\cpp_adams_dogv2\\l43.x_t");



//    // add joints //
//    //leg1
//    auto& lf_r1 = quad->addRevoluteJoint(lf_p1, body, leg_pe[0], std::array<double, 3>{-1, 0, 0}.data());
//    auto& lf_r2 = quad->addRevoluteJoint(lf_p2, lf_p1, leg_pe[1], std::array<double, 3>{0, 0, -1}.data());
//    auto& lf_r3 = quad->addRevoluteJoint(lf_p3, lf_p2, leg_pe[2], std::array<double, 3>{0, 0, -1}.data());
//    //leg2
//    auto& lr_r1 = quad->addRevoluteJoint(lr_p1, body, leg_pe[3], std::array<double, 3>{-1, 0, 0}.data());
//    auto& lr_r2 = quad->addRevoluteJoint(lr_p2, lr_p1, leg_pe[4], std::array<double, 3>{0, 0, -1}.data());
//    auto& lr_r3 = quad->addRevoluteJoint(lr_p3, lr_p2, leg_pe[5], std::array<double, 3>{0, 0, -1}.data());
//    //leg3
//    auto& rr_r1 = quad->addRevoluteJoint(rr_p1, body, leg_pe[6], std::array<double, 3>{1, 0, 0}.data());
//    auto& rr_r2 = quad->addRevoluteJoint(rr_p2, rr_p1, leg_pe[7], std::array<double, 3>{0, 0, 1}.data());
//    auto& rr_r3 = quad->addRevoluteJoint(rr_p3, rr_p2, leg_pe[8], std::array<double, 3>{0, 0, 1}.data());
//    //leg4
//    auto& rf_r1 = quad->addRevoluteJoint(rf_p1, body, leg_pe[9], std::array<double, 3>{1, 0, 0}.data());
//    auto& rf_r2 = quad->addRevoluteJoint(rf_p2, rf_p1, leg_pe[10], std::array<double, 3>{0, 0, 1}.data());
//    auto& rf_r3 = quad->addRevoluteJoint(rf_p3, rf_p2, leg_pe[11], std::array<double, 3>{0, 0, 1}.data());

//    // add motion //
//    //leg1
//    auto& lf_m1 = quad->addMotion(lf_r1);
//    auto& lf_m2 = quad->addMotion(lf_r2);
//    auto& lf_m3 = quad->addMotion(lf_r3);
//    //leg2
//    auto& lr_m1 = quad->addMotion(lr_r1);
//    auto& lr_m2 = quad->addMotion(lr_r2);
//    auto& lr_m3 = quad->addMotion(lr_r3);
//    //leg3
//    auto& rr_m1 = quad->addMotion(rr_r1);
//    auto& rr_m2 = quad->addMotion(rr_r2);
//    auto& rr_m3 = quad->addMotion(rr_r3);
//    //leg4
//    auto& rf_m1 = quad->addMotion(rf_r1);
//    auto& rf_m2 = quad->addMotion(rf_r2);
//    auto& rf_m3 = quad->addMotion(rf_r3);


//    // add end-effector //
//    auto body_ee_maki = body.addMarker("body_ee_mak_i");
//    auto body_ee_makj = quad->ground().addMarker("body_ee_mak_j");

//    auto& body_ee = quad->generalMotionPool().add<aris::dynamic::GeneralMotion>("body_ee", &body_ee_maki, &body_ee_makj);
//    auto& lf_ee = quad->addPointMotion(lf_p3, quad->ground(), ee_pos[0]);
//    quad->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
//    auto& lr_ee = quad->addPointMotion(lr_p3, quad->ground(), ee_pos[1]);
//    quad->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
//    auto& rr_ee = quad->addPointMotion(rr_p3, quad->ground(), ee_pos[2]);
//    quad->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
//    auto& rf_ee = quad->addPointMotion(rf_p3, quad->ground(), ee_pos[3]);
//    quad->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());


//    auto& inverse_kinematic_solver = quad->solverPool().add<aris::dynamic::InverseKinematicSolver>();
//    auto& forward_kinematic_solver = quad->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
//    auto& inverse_dynamic_solver = quad->solverPool().add<aris::dynamic::InverseDynamicSolver>();
//    auto& forward_dynamic_solver = quad->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

//    auto& stand_universal = quad->solverPool().add<aris::dynamic::UniversalSolver>();

//    //this->solverPool().add<aris::dynamic::UniversalSolver>();

//    //this->solverPool().add<aris::dynamic::UniversalSolver>();


//    // 添加仿真器和仿真结果 //
//    auto& adams = quad->simulatorPool().add<aris::dynamic::AdamsSimulator>();
//    auto& result = quad->simResultPool().add<aris::dynamic::SimResult>();

//    quad->init();

//    // 设置默认拓扑结构 //
//    for (auto& m : quad->motionPool())m.activate(true);
//    for (auto& gm : quad->generalMotionPool())gm.activate(false);

//    return quad;
//}
auto createControllerQuadruped()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

#ifdef UNIX

    auto &force_slave = controller->slavePool().add<aris::control::EthercatSlave>();
    force_slave.scanInfoForCurrentSlave();
    force_slave.scanPdoForCurrentSlave();

#endif // WIN32



    for (aris::Size i = 0; i < 12; ++i)
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
            0,0,0,0,0,0
        };
#else
        double pos_offset[12]
        {
            -0.0168354,
            -20554.9,
            1.75733,
            -0.0479177,
            1.0982,
            -0.853603,
            0.0203252,
            0.00080534,
            0.0109104,
            -0.617408,
            -7.2407,
            0.80465



        };
#endif
        double pos_factor[12]
        {
            16384.0 *20 / 2 / PI,16384.0 *20 / 2 / PI,16384.0 *20 / 2 / PI,
            16384.0 *20 / 2 / PI,16384.0 *20 / 2 / PI,16384.0 *20 / 2 / PI,
            16384.0 *20 / 2 / PI,16384.0 *20 / 2 / PI,16384.0 *20 / 2 / PI,
            16384.0 *20 / 2 / PI,16384.0 *20 / 2 / PI,16384.0 *20 / 2 / PI

        };
        double max_pos[12]
        {
            PI,PI,PI*2,
            PI,PI,PI*2,
            PI,PI,PI*2,
            PI,PI,PI*2,
        };
        double min_pos[12]
        {
            -PI,-PI,-PI*2,
            -PI,-PI,-PI*2,
            -PI,-PI,-PI*2,
            -PI,-PI,-PI*2,
        };
        double max_vel[12]
        {
            350 / 60 * 2 * PI, 350 / 60 * 2 * PI, 350 / 60 * 2 * PI,
            350 / 60 * 2 * PI, 350 / 60 * 2 * PI, 350 / 60 * 2 * PI,
            350 / 60 * 2 * PI, 350 / 60 * 2 * PI, 350 / 60 * 2 * PI,
            350 / 60 * 2 * PI, 350 / 60 * 2 * PI, 350 / 60 * 2 * PI


        };
        double max_acc[12]
        {
            30000,  30000,  30000,
            30000,  30000,  30000,
            30000,  30000,  30000,
            30000,  30000,  30000,
        };

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
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setVirtual(true);
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
    plan_root->planPool().add<DogTorqueControl>();
    plan_root->planPool().add<DogHome>();
    plan_root->planPool().add<DogSwitchPrePose>();
    plan_root->planPool().add<DogPrepare>();
    plan_root->planPool().add<DogUpDown>();
    plan_root->planPool().add<DogTaBu>();
    plan_root->planPool().add<DogForward>();
    plan_root->planPool().add<DogBack>();
    plan_root->planPool().add<DogLeft>();
    plan_root->planPool().add<DogRight>();
    plan_root->planPool().add<DogPitch>();
    plan_root->planPool().add<DogRoll>();
    plan_root->planPool().add<DogYaw>();


    // 正玄曲线 //
    plan_root->planPool().add<MoveJS>();
    return plan_root;
}

auto setStandTopologyIK(aris::server::ControlServer &cs)->void
{

    // 站立拓扑结构下，分配求解器内存 //
    // 失效就是不可规划，有效可以规划
    cs.model().generalMotionPool()[0].activate(false);//body
    cs.model().generalMotionPool()[1].activate(false);//leg1
    cs.model().generalMotionPool()[2].activate(false);//leg2
    cs.model().generalMotionPool()[3].activate(false);//leg3
    cs.model().generalMotionPool()[4].activate(false);//leg4

    for(int i=0;i<12;++i)
        cs.model().motionPool()[0].activate(true);
    cs.model().solverPool()[0].allocateMemory();
}
auto setTrotTopologyIK(aris::server::ControlServer& cs)->void
{

    // bound拓扑结构下，分配求解器内存 //
    cs.model().generalMotionPool()[0].activate(false);//body
    cs.model().generalMotionPool()[1].activate(false);//leg1
    cs.model().generalMotionPool()[2].activate(false);//leg2
    cs.model().generalMotionPool()[3].activate(false);//leg3
    cs.model().generalMotionPool()[4].activate(false);//leg4

    for (int i = 0; i < 12; ++i)
        cs.model().motionPool()[0].activate(true);
    cs.model().solverPool()[4].allocateMemory();

}
auto setBoundTopologyIK(aris::server::ControlServer& cs)->void
{

    // bound拓扑结构下，分配求解器内存 //
    cs.model().generalMotionPool()[0].activate(false);//body
    cs.model().generalMotionPool()[1].activate(false);//leg1
    cs.model().generalMotionPool()[2].activate(false);//leg2
    cs.model().generalMotionPool()[3].activate(false);//leg3
    cs.model().generalMotionPool()[4].activate(false);//leg4

    for (int i = 0; i < 12; ++i)
        cs.model().motionPool()[0].activate(true);
    cs.model().solverPool()[4].allocateMemory();

}


}
