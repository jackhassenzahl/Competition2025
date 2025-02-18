#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Gripper.h"

class GripperSetIntake : public frc2::CommandHelper<frc2::Command, GripperSetIntake>
{
    public:

        GripperSetIntake(units::angle::degree_t angle, double velocity, Gripper *gripper);

        void Execute()    override;
        bool IsFinished() override;

    private:

        units::angle::degree_t m_angle;
        double                 m_velocity;
        Gripper               *m_gripper;
};
