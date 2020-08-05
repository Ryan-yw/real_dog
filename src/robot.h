#ifndef ROBOT_H_
#define ROBOT_H_

#include <memory>
#include <aris.hpp>

namespace robot
{
    class SetMaxToq :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~SetMaxToq();
        explicit SetMaxToq(const std::string &name = "set_max_toq");
        ARIS_REGISTER_TYPE(SetMaxToq);
    };

    class DogReadJoint :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogReadJoint();
        explicit DogReadJoint(const std::string &name = "dog_read_joint");
        ARIS_REGISTER_TYPE(DogReadJoint);
    };

    class DogMoveJoint :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~DogMoveJoint();
        explicit DogMoveJoint(const std::string &name = "dog_move_joint");
        ARIS_REGISTER_TYPE(DogMoveJoint);
    private:
        double dir_;
    };
    
    class DogHome :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogHome();
        explicit DogHome(const std::string &name = "dog_home");
        ARIS_REGISTER_TYPE(DogHome);
    };

    class DogPrepare :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogPrepare();
        explicit DogPrepare(const std::string &name = "dog_prepare");
        ARIS_REGISTER_TYPE(DogPrepare);
    };

    class DogSitDown :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogSitDown();
        explicit DogSitDown(const std::string& name = "dog_sitdown");
        ARIS_REGISTER_TYPE(DogSitDown);
    };

    class DogStandUp :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogStandUp();
        explicit DogStandUp(const std::string& name = "dog_standup");
        ARIS_REGISTER_TYPE(DogStandUp);
    };

    class DogTaBu :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogTaBu();
        explicit DogTaBu(const std::string &name = "dog_tabu");
        ARIS_REGISTER_TYPE(DogTaBu);
    private:
        double step_;
    };

    class DogForward :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogForward();
        explicit DogForward(const std::string &name = "dog_forward");
        ARIS_REGISTER_TYPE(DogForward);
    private:
        double step_;
    };

    class DogBack :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogBack();
        explicit DogBack(const std::string& name = "dog_back");
        ARIS_REGISTER_TYPE(DogBack);
    private:
        double step_;
    };

    class DogLeft :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogLeft();
        explicit DogLeft(const std::string& name = "dog_left");
        ARIS_REGISTER_TYPE(DogLeft);
    private:
        double step_;
    };

    class DogRight :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogRight();
        explicit DogRight(const std::string& name = "dog_right");
        ARIS_REGISTER_TYPE(DogRight);
    private:
        double step_;
    };

    class DogPitchUp :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogPitchUp();
        explicit DogPitchUp(const std::string& name = "dog_pitchup");
        ARIS_REGISTER_TYPE(DogPitchUp);
    };

    class DogPitchDown :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogPitchDown();
        explicit DogPitchDown(const std::string& name = "dog_pitchdown");
        ARIS_REGISTER_TYPE(DogPitchDown);
    };

    class DogRolll :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogRolll();
        explicit DogRolll(const std::string& name = "dog_rolll");
        ARIS_REGISTER_TYPE(DogRolll);
    };

    class DogRollr :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogRollr();
        explicit DogRollr(const std::string& name = "dog_rollr");
        ARIS_REGISTER_TYPE(DogRollr);
    };

    class DogYawl :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogYawl();
        explicit DogYawl(const std::string& name = "dog_yawl");
        ARIS_REGISTER_TYPE(DogYawl);
    };

    class DogYawr :public aris::plan::Plan
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;

        virtual ~DogYawr();
        explicit DogYawr(const std::string& name = "dog_tawr");
        ARIS_REGISTER_TYPE(DogYawr);
    };

    auto createControllerQuadruped()->std::unique_ptr<aris::control::Controller>;
    auto createPlanQuadruped()->std::unique_ptr<aris::plan::PlanRoot>;
}

#endif
