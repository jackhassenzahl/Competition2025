#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Gripper.h"

#include <constants/xBoxConstants.h>
#include <constants/Extreme3DConstants.h>
#include <constants/ControllerConstants.h>
#include <constants/ControlPanelConstants.h>
#include <constants/CanConstants.h>

// #include "Constants.h"
#include "ConstantsPose.h"

enum GripperState
{
    ElevatorMove,
    ArmMove,
    GripperWheelsMove,
    Finish,
    Complete
};

struct GripperStateData
{
    units::meter_t         ElevatorOffset = 0_m;
    units::time::second_t  Wait1          = 0_s;
    units::angle::degree_t ArmOffset      = 0_deg;
    units::time::second_t  Wait2          = 0_s;
    units::voltage::volt_t GripperVoltage = 0_V;
    units::time::second_t  Wait3          = 0_s;

    units::meter_t         ElevatorFinish = 0_m;
    units::angle::degree_t ArmFinish      = 0_deg;
};

class GripperActivate : public frc2::CommandHelper<frc2::Command, GripperActivate>
{
    public:

        explicit GripperActivate(Gripper *gripper);

        void     Initialize() override;
        void     Execute()    override;
        bool     IsFinished() override;

    private:

        void CoralGround();
        void CoralStation();
        void CoralL123();
        void CoralL4();
        void AlgaeGround();
        void AlgaeOnCoral();
        void AlgaeLow();
        void AlgaeHigh();
        void AlgaeProcessor();
        void AlgaeBarge();

        GripperState          m_state;
        GripperStateData      m_stateData;

        units::time::second_t m_startTime;
        bool                  m_isFinished = false;

        GripperPoseEnum       m_gripperPose;  // The gripper pose
        Gripper              *m_gripper;      // The Gripper subsystem
};
