#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Leds.h"

#include "commands/SetLeds.h"

class AutonomousLed : public frc2::CommandHelper<frc2::SequentialCommandGroup, AutonomousLed>
{
    public:

        explicit AutonomousLed(Leds *m_leds);
};
