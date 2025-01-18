#include "commands/AutonomousLed.h"

AutonomousLed::AutonomousLed(Leds *leds)
{
    // Set the command name
    SetName("AutonomusLed");

    AddCommands(SetLeds(LedMode::Rainbow,   5_s, leds),
                SetLeds(LedMode::Strobe,    5_s, leds),
                SetLeds(LedMode::HvaColors, 5_s, leds));
}

