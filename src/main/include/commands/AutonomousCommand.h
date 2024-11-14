#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

class AutonomousCommand : public frc2::CommandHelper<frc2::Command, AutonomousCommand>
{
public:

    explicit AutonomousCommand();

    void     Initialize()          override;
    void     Execute()             override;
    bool     IsFinished()          override;
    void     End(bool interrupted) override;
    
    bool     RunsWhenDisabled()    const override;

private:
};
