#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Leds.h"
#include "subsystems/Drivetrain.h"

class AutonomousParallel : public frc2::CommandHelper<frc2::ParallelCommandGroup, AutonomousParallel>
{
    public:

        explicit AutonomousParallel(Leds *leds, Drivetrain *drivetrain);
};
