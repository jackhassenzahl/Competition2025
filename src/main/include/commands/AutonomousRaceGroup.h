#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelRaceGroup.h>

class AutonomousRaceGroup : public frc2::CommandHelper<frc2::ParallelRaceGroup, AutonomousRaceGroup>
{
    public:

        explicit AutonomousRaceGroup();
};
