#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Arm.h"

class ArmSetAngle : public frc2::CommandHelper<frc2::Command, ArmSetAngle>
{
    public:

        ArmSetAngle(units::angle::degree_t setArmAngleTo, Arm *arm);

        void Execute()    override;
        bool IsFinished() override;

    private:

        units::angle::degree_t m_angle;
        Arm                   *m_arm;
};
