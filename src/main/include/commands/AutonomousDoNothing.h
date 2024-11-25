#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

class AutonomousDoNothing : public frc2::CommandHelper<frc2::Command, AutonomousDoNothing>
{
    public:

        explicit AutonomousDoNothing();
};
