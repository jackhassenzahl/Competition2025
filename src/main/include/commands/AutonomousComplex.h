#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

class AutonomousComplex : public frc2::CommandHelper<frc2::SequentialCommandGroup, AutonomousComplex>
{
    public:

        explicit AutonomousComplex(Leds *leds, Drivetrain *drivetrain);
};
