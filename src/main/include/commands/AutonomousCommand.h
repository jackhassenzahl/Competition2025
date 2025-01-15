#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class AutonomousCommand : public frc2::CommandHelper<frc2::Command, AutonomousCommand>
{
    public:

        explicit AutonomousCommand();

        void     Initialize()          override;
        void     Execute()             override;
        void     End(bool interrupted) override;
        bool     IsFinished()          override;
};
