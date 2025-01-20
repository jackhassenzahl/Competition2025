#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "commands/AutonomousLed.h"
#include "commands/AutonomousParallel.h"
#include "commands/SetLeds.h"
#include "commands/ChassisDriveTime.h"

class AutonomousComplex : public frc2::CommandHelper<frc2::SequentialCommandGroup, AutonomousComplex>
{
    public:

        explicit AutonomousComplex(Leds *leds, Drivetrain *drivetrain);
};
