#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>

#include "robot.h"
#include"plan.h"

double input_angle[12] = {0};
double init_pos_angle[12] = { 0 };
std::string gait="trot";
std::string prepose="same";
//输出参数，模型曲线测试使用
double file_current_leg[12] = { 0 };
double file_current_body[16] = { 0 };
double time_test = 0;

using namespace aris::dynamic;
using namespace aris::plan;
const double PI = aris::PI;
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


    for (int i = 0; i < 1; ++i)
    {
        std::uint16_t max_toq=1000;
        this->ecController()->motionPool()[i].writePdo(0x6072,0,max_toq);
    }

//    for (int i = 0; i < 12; ++i)
//    {
//        cout << this->ecController()->motionPool()[i].actualPos() <<"  ";
//    }
//    cout << std::endl;

    //返回0表示正常结束，返回负数表示报错，返回正数表示正在执行
    return 1000 - count();
}
auto SetMaxToq::collectNrt()->void {}
SetMaxToq::~SetMaxToq() = default;
SetMaxToq::SetMaxToq(const std::string &name){
    aris::core::fromXmlString(command(),
                "<Command name=\"set_toq\"/>");
}


//read foot force
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
//    begin_angle[1] = controller()->motionPool()[1].actualPos();
//    begin_angle[2] = controller()->motionPool()[2].actualPos();
//    begin_angle[3] = controller()->motionPool()[3].actualPos();
//    begin_angle[4] = controller()->motionPool()[4].actualPos();
//    begin_angle[5] = controller()->motionPool()[5].actualPos();
//    begin_angle[6] = controller()->motionPool()[6].actualPos();
//    begin_angle[7] = controller()->motionPool()[7].actualPos();
//    begin_angle[8] = controller()->motionPool()[8].actualPos();
//    begin_angle[9] = controller()->motionPool()[9].actualPos();
//    begin_angle[10] = controller()->motionPool()[10].actualPos();
//    begin_angle[11] = controller()->motionPool()[11].actualPos();

    for(int i=0;i<1;i++)
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

//read joint qorque
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
    EllipseTrajectory e1(400, 100, 0, s1);

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
        mout() << "input erdog_ror" << std::endl;
    }

    for (int j = 0; j < 12; j++)
    {
        mout() << init_pos_angle[j] << std::endl;
    }
    
    //输出角度，用于仿真测试
    //{
    //    输出电机角度
    //    for (int i = 0; i < 12; i++)
    //    {
    //        lout() << input_angle[i] << "\t";
    //    }
    //    time_test += 0.001;
    //    lout() << time_test << "\t";

    //    输出身体和足尖曲线
    //    for (int j = 0; j < 12; j++)
    //    {
    //        lout() << file_current_leg[j] << "\t";
    //    }
    //    lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    //}

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

    for(int i=0;i<12;i++)
    {
        angle[i] = begin_angle[i] + (0-begin_angle[i]) * s1.getTCurve(count());
    }

    for(int i=0;i<12;i++)
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
        for (int i = 0; i < 12; i++)
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
        begin_angle[2] = controller()->motionPool()[2].actualPos()/1.5;
        begin_angle[3] = controller()->motionPool()[3].actualPos();
        begin_angle[4] = controller()->motionPool()[4].actualPos();
        begin_angle[5] = controller()->motionPool()[5].actualPos()/1.5;
        begin_angle[6] = controller()->motionPool()[6].actualPos();
        begin_angle[7] = controller()->motionPool()[7].actualPos();
        begin_angle[8] = controller()->motionPool()[8].actualPos()/1.5;
        begin_angle[9] = controller()->motionPool()[9].actualPos();
        begin_angle[10] = controller()->motionPool()[10].actualPos();
        begin_angle[11] = controller()->motionPool()[11].actualPos()/1.5;
    }


    TCurve s1(5, 2);//0.9s
    s1.getCurveParam();

    if (prepose_ == "same")//same
    {
        for (int i = 0; i < 12; i++)
        {
            angle[i] = begin_angle[i] + (distance_same[i] - begin_angle[i]) * s1.getTCurve(count());
        }
    }
    else if(prepose_=="symmetry")//symmetry
    {
        for (int i = 0; i < 12; i++)
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
        for (int i = 0; i < 12; i++)
        {
            lout() << angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";
        //输出身体和足尖曲线
        for (int j = 0; j < 12; j++)
        {
            lout() << file_current_leg[j] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; i++)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(1.5 * angle[i]);
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
    double distance[12] = { 0 };
 
    for (int i = 0; i < 12; i++)
    {
        distance[i] = init_pos_angle[i];
    }

   

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

    for(int i=0;i<12;i++)
    {
        angle[i] = begin_angle[i] + distance[i] * s1.getTCurve(count());
    }

    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; i++)
        {
            lout() << angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";
        //输出身体和足尖曲线
        for (int j = 0; j < 12; j++)
        {
            lout() << file_current_leg[j] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; i++)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(1.5 * angle[i]);
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

    


    //步态规划
    if (prepose == "same")// same
    {
        ret = updownPlanSame(count() - 1, &e1, distance_, input_angle);
    }
    else if (prepose == "symmetry")  //symmetry
    {
        ret = updownPlanSymmetry(count() - 1, &e1, distance_, input_angle);
    }
    else //
    {
        mout() << "input error" << std::endl;
    }


    //输出角度，用于仿真测试
    {
        for (int i = 0; i < 12; i++)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int j = 0; j < 12; j++)
        {
            lout() << file_current_leg[j] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; i++)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(1.5 * input_angle[i]);
        else
            controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    return ret;
}
DogUpDown::DogUpDown(const std::string& name) 
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_updown\">"
        "	<Param name=\"distance\" default=\"1\" abbreviation=\"d\"/>"
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
    EllipseTrajectory e1(0, 100, 0, s1);
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
        for (int i = 0; i < 12; i++)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int j = 0; j < 12; j++)
        {
            lout() << file_current_leg[j] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; i++)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(1.5 * input_angle[i]);
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

    TCurve s1(1, 4);
    s1.getCurveParam();
    EllipseTrajectory e1(400, 100, 0, s1);

    //步态规划
    if (gait == "trot" && prepose == "same")//trot & same
    {
        ret = trotPlanSameLeg(step_, count() - 1, &e1, input_angle);
    }
    else if (gait =="walk"  && prepose == "same") //walk & same
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
        for (int i = 0; i < 12; i++)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int j = 0; j < 12; j++)
        {
            lout() << file_current_leg[j] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; i++)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(1.5 * input_angle[i]);
        else
            controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }

    return ret;
}
DogForward::DogForward(const std::string &name) 
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_forward\">"
        "	<Param name=\"step\" default=\"1\" abbreviation=\"n\"/>"
        "</Command>");
}
DogForward::~DogForward() = default;

//后退
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
    EllipseTrajectory e1(-400, 100, 0, s1);
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
        for (int i = 0; i < 12; i++)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int j = 0; j < 12; j++)
        {
            lout() << file_current_leg[j] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; i++)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(1.5 * input_angle[i]);
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
    EllipseTrajectory e1(0, 100, -200, s1);
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
        for (int i = 0; i < 12; i++)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int j = 0; j < 12; j++)
        {
            lout() << file_current_leg[j] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; i++)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(1.5 * input_angle[i]);
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

    EllipseTrajectory e1(0, 100,200, s1);
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
        for (int i = 0; i < 12; i++)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int j = 0; j < 12; j++)
        {
            lout() << file_current_leg[j] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; i++)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(1.5 * input_angle[i]);
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
        for (int i = 0; i < 12; i++)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int j = 0; j < 12; j++)
        {
            lout() << file_current_leg[j] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; i++)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(1.5 * input_angle[i]);
        else
            controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    return ret;
}
DogPitch::DogPitch(const std::string& name) 
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_pitch\">"
        "	<Param name=\"angle\" default=\"10\" abbreviation=\"d\"/>"
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
        for (int i = 0; i < 12; i++)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int j = 0; j < 12; j++)
        {
            lout() << file_current_leg[j] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; i++)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(1.5 * input_angle[i]);
        else
            controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    return ret;
}
DogRoll::DogRoll(const std::string& name) 
{
    aris::core::fromXmlString(command(),
        "<Command name=\"dog_roll\">"
        "	<Param name=\"angle\" default=\"10\" abbreviation=\"d\"/>"
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
        for (int i = 0; i < 12; i++)
        {
            lout() << input_angle[i] << "\t";
        }
        time_test += 0.001;
        lout() << time_test << "\t";

        //输出身体和足尖曲线
        for (int j = 0; j < 12; j++)
        {
            lout() << file_current_leg[j] << "\t";
        }
        lout() << file_current_body[3] << "\t" << file_current_body[7] << "\t" << file_current_body[11] << std::endl;
    }
    //发送电机角度
    for (int i = 0; i < 12; i++)
    {
        if (i == 2 || i == 5 || i == 8 || i == 11)
            controller()->motionPool()[i].setTargetPos(1.5 * input_angle[i]);
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


auto createControllerQuadruped()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);
    auto &force_slave = controller->slavePool().add<aris::control::EthercatSlave>();
    force_slave.scanInfoForCurrentSlave();
    force_slave.scanPdoForCurrentSlave();

    for (aris::Size i = 0; i < 1; ++i)
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
            -20556.8


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
           1000* PI,PI,PI,
            PI,PI,PI,
            PI,PI,PI,
            PI,PI,PI,
        };
        double min_pos[12]
        {
            -PI*1000,-PI,-PI,
            -PI,-PI,-PI,
            -PI,-PI,-PI,
            -PI,-PI,-PI,
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
            3000,  3000,  3000,
            3000,  3000,  3000,
            3000,  3000,  3000,
            3000,  3000,  3000,
        };

        int phy_id[12]={1,1,0,3,4,5,8,7,6,9,10,11};


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

    //自己写的命令
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
    return plan_root;
}

}
