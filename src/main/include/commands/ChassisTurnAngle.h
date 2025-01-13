#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class ChassisTurnAngle : public frc2::CommandHelper<frc2::Command, ChassisTurnAngle>
{
    public:

        explicit ChassisTurnAngle();

        void     Initialize()          override;
        void     Execute()             override;
        void     End(bool interrupted) override;
        bool     IsFinished()          override;
};
