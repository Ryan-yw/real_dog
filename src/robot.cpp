#include <algorithm>
#include <array>
#include <stdlib.h>
#include <string>
#include <bitset>

#include "robot.h"
#include"plan.h"

double input_angle[12] = {0};
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


    for (int i = 0; i < 12; ++i)
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
SetMaxToq::SetMaxToq(const std::string &name) : Plan(name){
    command().loadXmlStr(
                "<Command name=\"set_toq\"/>");
}

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
 //   begin_angle[10] = controller()->motionPool()[10].actualPos();
 //   begin_angle[11] = controller()->motionPool()[11].actualPos();

    for(int i=0;i<12;i++)
    {
       std::cout<<begin_angle[i]<<std::endl;
    }

    return 0;
}
DogReadJoint::DogReadJoint(const std::string &name) : Plan(name)
{
    command().loadXmlStr(
       "<Command name=\"dog_readj\"/>");
}
DogReadJoint::~DogReadJoint() = default;

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
//        begin_angle[0] = controller()->motionPool()[0].targetPos();
//        begin_angle[1] = controller()->motionPool()[1].targetPos();
//        begin_angle[2] = controller()->motionPool()[2].targetPos();
//        begin_angle[3] = controller()->motionPool()[3].targetPos();
//        begin_angle[4] = controller()->motionPool()[4].targetPos();
//        begin_angle[5] = controller()->motionPool()[5].targetPos();
        begin_angle[6] = controller()->motionPool()[6].targetPos();
        begin_angle[7] = controller()->motionPool()[7].targetPos();
        begin_angle[8] = controller()->motionPool()[8].targetPos();
//        begin_angle[9] = controller()->motionPool()[9].targetPos();
//        begin_angle[10] = controller()->motionPool()[10].targetPos();
//        begin_angle[11] = controller()->motionPool()[11].targetPos();
    }
        TCurve s1(1,3);
        s1.getCurveParam();


//    TCurve s1(5, 2);
//    s1.getCurveParam();
//    EllipseTrajectory e1(-1, 0.8, 0, s1);
//    e1.getEllipseTrajectory(count());
//    pe[0] += dir_*0.1 * e1.z_;
//    pe[1] += dir_*0.1 * e1.x_;
//    pe[2] += 0.1 * e1.y_;

//    double angle0 = begin_angle[0] + dir_ * 0.1 * s1.getTCurve(count());
//    double angle1 = begin_angle[1] + dir_ * 0.1 * s1.getTCurve(count());
//    double angle2 = begin_angle[2] + dir_ * 0.1 * s1.getTCurve(count());
//    double angle3 = begin_angle[3] + dir_ * 0.1 * s1.getTCurve(count());
//    double angle4 = begin_angle[4] + dir_ * 0.1 * s1.getTCurve(count());
//    double angle5 = begin_angle[5] + dir_ * 0.1 * s1.getTCurve(count());
    double angle6 = begin_angle[6] + dir_ * 0.1 * s1.getTCurve(count());
    double angle7 = begin_angle[7] + dir_ * 0.1 * s1.getTCurve(count());
    double angle8 = begin_angle[8] + dir_ * 0.1 * s1.getTCurve(count());
//    double angle9 = begin_angle[9] + dir_ * 0.1 * s1.getTCurve(count());
//    double angle10 = begin_angle[10] + dir_ * 0.1 * s1.getTCurve(count());
//    double angle11 = begin_angle[11] + dir_ * 0.1 * s1.getTCurve(count());



//    controller()->motionPool()[0].setTargetPos(angle0);
//    controller()->motionPool()[1].setTargetPos(angle1);
//    controller()->motionPool()[2].setTargetPos(angle2);
//    controller()->motionPool()[3].setTargetPos(angle3);
//    controller()->motionPool()[4].setTargetPos(angle4);
//    controller()->motionPool()[5].setTargetPos(angle5);
    controller()->motionPool()[6].setTargetPos(angle6);
    controller()->motionPool()[7].setTargetPos(angle7);
    controller()->motionPool()[8].setTargetPos(angle8);
//    controller()->motionPool()[6].setTargetPos(angle9);
//    controller()->motionPool()[7].setTargetPos(angle10);
//    controller()->motionPool()[8].setTargetPos(angle11);
    std::cout <<s1.getTc() << std::endl;
    return s1.getTc() * 1000-count();
}
auto DogMoveJoint::collectNrt()->void {}
DogMoveJoint::DogMoveJoint(const std::string &name) : Plan(name)
{
    command().loadXmlStr(
       "<Command name=\"dog_mvj\">"
        "	<Param name=\"direction\" default=\"1\" abbreviation=\"d\"/>"
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
DogHome::DogHome(const std::string &name) : Plan(name)
{
    command().loadXmlStr(
       "<Command name=\"dog_home\"/>");
}
DogHome::~DogHome() = default;

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
    const double distance[12]={
                                0,0.582289,-1.12207,
                                0,0.582289,-1.12207,
                                0,-0.582289,1.12207,
                                0,-0.582289,1.12207
                                //0,0.666031,-1.10885,
                                //0,0.484712,-1.10885,
                                //0,-0.484712,1.10885,
                                //0,-0.666031,1.10885
                              };
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


    TCurve s1(5,2);//0.9s
    s1.getCurveParam();

    for(int i=0;i<12;i++)
    {
        angle[i] = begin_angle[i] + distance[i] * s1.getTCurve(count());
    }

    for(int i=0;i<12;i++)
    {
        lout() << angle[i] << "\t";
        controller()->motionPool()[i].setTargetPos(angle[i]);
    }
    time_test += 0.001;
    lout() << time_test << std::endl;

    return s1.getTc() * 1000-count();

}
DogPrepare::DogPrepare(const std::string &name) : Plan(name)
{
    command().loadXmlStr(
       "<Command name=\"dog_prepare\"/>");
}
DogPrepare::~DogPrepare() = default;

//下蹲
auto DogSitDown::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogSitDown::executeRT()->int
{

    if (count() == 1)this->master()->logFileRawName("sitdown");

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

    ret = downPlan(count()-1, &e1, input_angle);

    for (int i = 0; i < 12; i++)
    {
        lout() << input_angle[i] << "\t";
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    time_test += 0.001;
    lout() << time_test << std::endl;
    return ret;
}
DogSitDown::DogSitDown(const std::string& name) : Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"dog_sitdown\"/>");
}
DogSitDown::~DogSitDown() = default;

//起立
auto DogStandUp::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogStandUp::executeRT()->int
{

    if (count() == 1)this->master()->logFileRawName("standup");

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

    ret = upPlan(count() - 1, &e1, input_angle);

    for (int i = 0; i < 12; i++)
    {
        lout() << input_angle[i] << "\t";
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    time_test += 0.001;
    lout() << time_test << std::endl;
    return ret;
}
DogStandUp::DogStandUp(const std::string& name) : Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"dog_standup\"/>");
}
DogStandUp::~DogStandUp() = default;

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
    EllipseTrajectory e1(0, 150, 0, s1);
    int ret = 1;
    ret = walkPlan(step_, count()-1, &e1, input_angle);

    for(int i=0;i<12;i++)
    {
        lout() << input_angle[i] << "\t";
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    time_test += 0.001;
    lout() << time_test << std::endl;
    return ret;
}
DogTaBu::DogTaBu(const std::string& name) : Plan(name)
{
    command().loadXmlStr(
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
    EllipseTrajectory e1(220, 150, 0, s1);
    ret =  walkPlan(step_,count()-1, &e1,input_angle);

    for(int i=0;i<12;i++)
    {
        lout() << input_angle[i] << "\t";
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    time_test += 0.001;
    lout() << time_test << std::endl;
    return ret;
}
DogForward::DogForward(const std::string &name) : Plan(name)
{
    command().loadXmlStr(
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
    EllipseTrajectory e1(-220, 150, 0, s1);
    ret = walkPlan(step_, count() - 1, &e1, input_angle);

    for (int i = 0; i < 12; i++)
    {
        lout() << input_angle[i] << "\t";
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    time_test += 0.001;
    lout() << time_test << std::endl;
    return ret;
}
DogBack::DogBack(const std::string& name) : Plan(name)
{
    command().loadXmlStr(
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
    if (count() == 1)this->master()->logFileRawName("left");
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
    EllipseTrajectory e1(0, 150, -220, s1);
    ret = walkPlan(step_, count() - 1, &e1, input_angle);

    for (int i = 0; i < 12; i++)
    {
        lout() << input_angle[i] << "\t";
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    time_test += 0.001;
    lout() << time_test << std::endl;
    return ret;
}
DogLeft::DogLeft(const std::string& name) : Plan(name)
{
    command().loadXmlStr(
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
    if (count() == 1)this->master()->logFileRawName("right");
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
    EllipseTrajectory e1(0, 150,220, s1);
    ret = walkPlan(step_, count() - 1, &e1, input_angle);

    for (int i = 0; i < 12; i++)
    {
        lout() << input_angle[i] << "\t";
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    time_test += 0.001;
    lout() << time_test << std::endl;
    return ret;
}
DogRight::DogRight(const std::string& name) : Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"dog_right\">"
        "	<Param name=\"step\" default=\"1\" abbreviation=\"n\"/>"
        "</Command>");
}
DogRight::~DogRight() = default;

//pitchup
auto DogPitchUp::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogPitchUp::executeRT()->int
{
    if (count() == 1)this->master()->logFileRawName("pitchup");
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
        //input_angle[10] = controller()->motionPool()[10].actualPos();
        //input_angle[11] = controller()->motionPool()[11].actualPos();
    }

    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);
    BodyPose body_pose(0, 0, 30, s1);

    ret = posePlan(count()-1, &e1, &body_pose, input_angle);

    for (int i = 0; i < 12; i++)
    {
        lout() << input_angle[i] << "\t";
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    time_test += 0.001;
    lout() << time_test << std::endl;
    return ret;
}
DogPitchUp::DogPitchUp(const std::string& name) : Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"dog_pitchup\">"
        "</Command>");
}
DogPitchUp::~DogPitchUp() = default;

//pitchdown
auto DogPitchDown::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogPitchDown::executeRT()->int
{
    if (count() == 1)this->master()->logFileRawName("pitchdown");
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
        //input_angle[10] = controller()->motionPool()[10].actualPos();
        //input_angle[11] = controller()->motionPool()[11].actualPos();
    }

    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);
    BodyPose body_pose(0, 0, -30, s1);

    ret = posePlan(count()-1, &e1, &body_pose, input_angle);

    for (int i = 0; i < 12; i++)
    {
        lout() << input_angle[i] << "\t";
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    time_test += 0.001;
    lout() << time_test << std::endl;
    return ret;
}
DogPitchDown::DogPitchDown(const std::string& name) : Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"dog_pitchdown\">"
        "</Command>");
}
DogPitchDown::~DogPitchDown() = default;


//rolll
auto DogRolll::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogRolll::executeRT()->int
{
    if (count() == 1)this->master()->logFileRawName("rolll");
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
        //input_angle[10] = controller()->motionPool()[10].actualPos();
        //input_angle[11] = controller()->motionPool()[11].actualPos();
    }

    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);
    BodyPose body_pose(30, 0, 0, s1);
    ret = posePlan(count()-1, &e1, &body_pose, input_angle);

    for (int i = 0; i < 12; i++)
    {
        lout() << input_angle[i] << "\t";
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    time_test += 0.001;
    lout() << time_test << std::endl;
    return ret;
}
DogRolll::DogRolll(const std::string& name) : Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"dog_rolll\">"
        "</Command>");
}
DogRolll::~DogRolll() = default;

//rollr
auto DogRollr::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogRollr::executeRT()->int
{
    if (count() == 1)this->master()->logFileRawName("rollr");
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
        //input_angle[10] = controller()->motionPool()[10].actualPos();
        //input_angle[11] = controller()->motionPool()[11].actualPos();
    }

    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);
    BodyPose body_pose(-30, 0, 0, s1);
    ret = posePlan(count()-1, &e1, &body_pose, input_angle);

    for (int i = 0; i < 12; i++)
    {
        lout() << input_angle[i] << "\t";
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    time_test += 0.001;
    lout() << time_test << std::endl;
    return ret;
}
DogRollr::DogRollr(const std::string& name) : Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"dog_rollr\">"
        "</Command>");
}
DogRollr::~DogRollr() = default;

//yawl
auto DogYawl::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogYawl::executeRT()->int
{
    if (count() == 1)this->master()->logFileRawName("yawl");
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
        //input_angle[10] = controller()->motionPool()[10].actualPos();
        //input_angle[11] = controller()->motionPool()[11].actualPos();
    }

    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);
    BodyPose body_pose(0, 30, 0, s1);
    ret = posePlan(count()-1, &e1, &body_pose, input_angle);

    for (int i = 0; i < 12; i++)
    {
        lout() << input_angle[i] << "\t";
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    time_test += 0.001;
    lout() << time_test << std::endl;
    return ret;
}
DogYawl::DogYawl(const std::string& name) : Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"dog_yawl\">"
        "</Command>");
}
DogYawl::~DogYawl() = default;

//yawr
auto DogYawr::prepareNrt()->void
{
    for (auto& m : motorOptions()) m = aris::plan::Plan::NOT_CHECK_ENABLE;
}
auto DogYawr::executeRT()->int
{
    if (count() == 1)this->master()->logFileRawName("yawr");
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
        //input_angle[10] = controller()->motionPool()[10].actualPos();
        //input_angle[11] = controller()->motionPool()[11].actualPos();
    }

    TCurve s1(5, 2);
    s1.getCurveParam();
    EllipseTrajectory e1(0, 0, 0, s1);
    BodyPose body_pose(0, -30, 0, s1);
    ret = posePlan(count()-1, &e1, &body_pose, input_angle);

    for (int i = 0; i < 12; i++)
    {
        lout() << input_angle[i] << "\t";
        controller()->motionPool()[i].setTargetPos(input_angle[i]);
    }
    time_test += 0.001;
    lout() << time_test << std::endl;
    return ret;
}
DogYawr::DogYawr(const std::string& name) : Plan(name)
{
    command().loadXmlStr(
        "<Command name=\"dog_yawr\">"
        "</Command>");
}
DogYawr::~DogYawr() = default;


auto createControllerQuadruped()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

    for (aris::Size i = 0; i < 12; ++i)
    {
#ifdef ARIS_USE_ETHERCAT_SIMULATION
        double pos_offset[12]
        {
            0,0,0,0,0,0
        };
#else
        double pos_offset[12]
        {
            //-0.0405786,
            //-0.128355,
            //1.11861,
            //-0.270588,
            //0.429535,
            //1.45398,
            //-0.0726683,
            //0.208502,
            //0.0887512,
            //0.911768,
            //0,
            //0

        };
#endif
        double pos_factor[12]
        {
            262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI,
            262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI,
            262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI,
            262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI, 262144.0 * 6 / 2 / PI,
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
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
        };
        double max_acc[12]
        {
            3000,  3000,  3000,
            3000,  3000,  3000,
            3000,  3000,  3000,
            3000,  3000,  3000,
        };

        int phy_id[12]={2,1,0,3,4,5,8,7,6,9,10,11};


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

        controller->slavePool().add<aris::control::EthercatMotor>().loadXmlStr(xml_str);

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
    plan_root->planPool().add<DogMoveJoint>();
    plan_root->planPool().add<DogHome>();
    plan_root->planPool().add<DogPrepare>();
    plan_root->planPool().add<DogSitDown>();
    plan_root->planPool().add<DogStandUp>();
    plan_root->planPool().add<DogTaBu>();
    plan_root->planPool().add<DogForward>();
    plan_root->planPool().add<DogBack>();
    plan_root->planPool().add<DogLeft>();
    plan_root->planPool().add<DogRight>();
    plan_root->planPool().add<DogPitchUp>();
    plan_root->planPool().add<DogPitchDown>();
    plan_root->planPool().add<DogRolll>();
    plan_root->planPool().add<DogRollr>();
    plan_root->planPool().add<DogYawl>();
    plan_root->planPool().add<DogYawr>();
    return plan_root;
}

}
