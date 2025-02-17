#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/InstantCommand.h>

#include "subsystems/Arm.h"
#include "subsystems/Grabber.h"

class ArmActivate : public frc2::CommandHelper<frc2::Command, ArmActivate>
{
    public:

        ArmActivate(units::angle::degree_t angleOffset, double grabberVoltage, Grabber *grabber, Arm *arm);

        void Execute()    override;
        bool IsFinished() override;

    private:

        units::angle::degree_t m_angleOffset;
        double                 m_grabberVoltage;
        Grabber               *m_grabber;
        Arm                   *m_arm;
};
