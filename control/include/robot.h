#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

#ifdef UNIX
#include<lcm/lcm-cpp.hpp>
#include<lcm/lcm.h>
#include<../lcm/state_estimator_lcmt.hpp>
#include<../lcm/wbc_test_data_t.hpp>
#endif // UNIX


namespace robot
{

    //class TestFilter : public aris::core::CloneObject<TestFilter, aris::plan::Plan>
    //{
    //public:
    //    auto virtual prepareNrt()->void;
    //    auto virtual executeRT()->int;
    //    auto virtual collectNrt()->void;

    //    virtual ~TestFilter();
    //    explicit TestFilter(const std::string& name = "test_filter");
    //};


    class SetMaxToq : public aris::core::CloneObject<SetMaxToq, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~SetMaxToq();
        explicit SetMaxToq(const std::string &name = "set_max_toq");
    };

    class DogReadForce :public aris::core::CloneObject<DogReadForce, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogReadForce();
        explicit DogReadForce(const std::string &name = "dog_read_force");

    };

    class DogReadJoint :public aris::core::CloneObject<DogReadJoint, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogReadJoint();
        explicit DogReadJoint(const std::string &name = "dog_read_joint");
    };

    class DogTorqueControl :public aris::core::CloneObject<DogTorqueControl, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogTorqueControl();
        explicit DogTorqueControl(const std::string &name = "dog_read_torque");
    private:
        double dir_;
    };

    class DogInitPos :public aris::core::CloneObject<DogInitPos, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogInitPos();
        explicit DogInitPos(const std::string& name = "dog_get_init_pos");
    };

    class DogSetWalkMode :public aris::core::CloneObject<DogSetWalkMode, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogSetWalkMode();
        explicit DogSetWalkMode(const std::string& name = "dog_set_walk_mode");
    private:
        std::string gait_;
        std::string prepose_;
    };

    class DogMoveJoint :public aris::core::CloneObject<DogMoveJoint, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~DogMoveJoint();
        explicit DogMoveJoint(const std::string &name = "dog_move_joint");
    private:
        double dir_;
    };

    class DogHome :public aris::core::CloneObject<DogHome, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogHome();
        explicit DogHome(const std::string &name = "dog_home");
    };

    ////class DogSwitchPrePose :public aris::core::CloneObject<DogSwitchPrePose, aris::plan::Plan>
    ////{
    ////public:
    ////    auto virtual prepareNrt()->void;
    ////    auto virtual executeRT()->int;

    ////    virtual ~DogSwitchPrePose();
    ////    explicit DogSwitchPrePose(const std::string& name = "dog_switchpose");
    ////private:
    ////    std::string prepose_;
    ////};

    class DogPrepare :public aris::core::CloneObject<DogPrepare, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int override;

        virtual ~DogPrepare();
        explicit DogPrepare(const std::string &name = "dog_prepare");
    };



    class DogUpDown :public aris::core::CloneObject<DogUpDown, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogUpDown();
        explicit DogUpDown(const std::string& name = "dog_updown");
    private:
        double distance_;
    };

    class DogTaBu :public aris::core::CloneObject<DogTaBu, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogTaBu();
        explicit DogTaBu(const std::string &name = "dog_tabu");
    private:
        double step_;
    };

    class DogForward :public aris::core::CloneObject<DogForward, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogForward();
        explicit DogForward(const std::string &name = "dog_forward");
    private:
        double step_;
        double z_;
        double fk_input_joint_[12] = {0};
        double leg_ee_[12] = {0};
    };

    class DogBack :public aris::core::CloneObject<DogBack, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogBack();
        explicit DogBack(const std::string& name = "dog_back");
    private:
        double step_;
    };

    class DogLeft :public aris::core::CloneObject<DogLeft, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogLeft();
        explicit DogLeft(const std::string& name = "dog_left");
    private:
        double step_;
    };

    class DogRight :public aris::core::CloneObject<DogRight, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogRight();
        explicit DogRight(const std::string& name = "dog_right");
    private:
        double step_;
    };

    class DogTurn :public aris::core::CloneObject<DogTurn, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogTurn();
        explicit DogTurn(const std::string& name = "dog_turn");
    private:
        double step_;
        double turn_angle_;
    };

    class DogPitch :public aris::core::CloneObject<DogPitch, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogPitch();
        explicit DogPitch(const std::string& name = "dog_pitch");
    private:
        double turn_angle_;
    };


    class DogRoll :public aris::core::CloneObject<DogRoll, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogRoll();
        explicit DogRoll(const std::string& name = "dog_roll");
    private:
        double turn_angle_;
    };


    class DogYaw :public aris::core::CloneObject<DogYaw, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogYaw();
        explicit DogYaw(const std::string& name = "dog_yaw");
    private:
        double turn_angle_;
    };

    class DogPose :public aris::core::CloneObject<DogPose, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogPose();
        explicit DogPose(const std::string& name = "dog_pose");
    private:
        double turn_angle_;
    };

    // cpp和adams测试 //
    class DogDynamicTest :public aris::core::CloneObject<DogDynamicTest, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int override;
  
        virtual ~DogDynamicTest();
        explicit DogDynamicTest(const std::string& name = "dog_dynamic");

    };

    class DogRun :public aris::core::CloneObject<DogRun, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogRun();
        explicit DogRun(const std::string &name = "dog_run");
    private:
        double step_;
        double z_;
    };
#ifdef UNIX

    class RobotDataProcess
    {
    public:

        RobotDataProcess();
        ~RobotDataProcess() = default;

        auto estimaterDataProcess(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const state_estimator_lcmt* msg)->void;
        auto wbcDataProcess(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const wbc_test_data_t* msg)->void;

    private:
        lcm::LCM lcm_robot_estimator_;
        std::thread estimator_data_robot_thread_;
        lcm::LCM lcm_robot_wbc_;
        std::thread wbc_data_robot_thread_;

    };

    // test thread
    class ControlThread :public aris::core::CloneObject<ControlThread, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~ControlThread();
        explicit ControlThread(const std::string& name = "test_thread");
    private:
        double dir_;

    };
#endif // UNIX



    auto createModelQuadruped()->std::unique_ptr<aris::dynamic::Model>;
    auto createControllerQuadruped()->std::unique_ptr<aris::control::Controller>;
    auto createPlanQuadruped()->std::unique_ptr<aris::plan::PlanRoot>;

}

#endif
