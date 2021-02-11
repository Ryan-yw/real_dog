#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace robot
{
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

    class DogSwitchPrePose :public aris::core::CloneObject<DogSwitchPrePose, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogSwitchPrePose();
        explicit DogSwitchPrePose(const std::string& name = "dog_switchpose");
    private:
        std::string prepose_;
    };

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



    auto createControllerQuadruped()->std::unique_ptr<aris::control::Controller>;
    auto createPlanQuadruped()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
