#pragma once

#include "subsystems/Leds.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

class SetLeds : public frc2::CommandHelper<frc2::Command, SetLeds>
{
    public:

        explicit SetLeds(int Mode, Leds *m_leds);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;
        bool     RunsWhenDisabled()    const override;

    private:

        int   m_mode;  // The LED mode for the command

        Leds *m_leds;  // Pointer to the LED subsystem class
};
