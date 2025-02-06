#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climb.h"

class ClimbSetAngle : public frc2::CommandHelper<frc2::Command, ClimbSetAngle>
{
    public:
        ClimbSetAngle(units::angle::degree_t setClimbAngleTo, Climb *climb);

        void Execute()    override;
        bool IsFinished() override;

    private:
        units::angle::degree_t m_angle;
        Climb                 *m_climb;
};
