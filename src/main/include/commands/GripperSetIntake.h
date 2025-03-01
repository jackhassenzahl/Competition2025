#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Gripper.h"

class GripperSetIntake : public frc2::CommandHelper<frc2::Command, GripperSetIntake>
{
    public:

        explicit GripperSetIntake(units::angle::degree_t angle, units::voltage::volt_t voltage, Gripper *gripper);

        void     Execute()    override;
        bool     IsFinished() override;

    private:

        units::angle::degree_t m_angle;
        units::voltage::volt_t m_voltage;
        Gripper               *m_gripper;
};
