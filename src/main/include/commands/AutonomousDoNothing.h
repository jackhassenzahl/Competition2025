#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class AutonomousDoNothing : public frc2::CommandHelper<frc2::Command, AutonomousDoNothing>
{
    public:

        explicit AutonomousDoNothing();

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;
    
        bool     RunsWhenDisabled()    const override;

    private:
};
