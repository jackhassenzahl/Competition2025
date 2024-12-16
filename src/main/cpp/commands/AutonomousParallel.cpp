#include "commands/SetLeds.h"
#include "commands/DriveTime.h"

#include "commands/AutonomousParallel.h"

AutonomousParallel::AutonomousParallel(Leds *leds, Drivetrain *drivetrain)
{
    // Set the command name
    SetName("AutonomousParallel");

    AddCommands(DriveTime(2_s, 0.5, drivetrain), SetLeds(LedMode::Rainbow, 5_s, leds));
}
