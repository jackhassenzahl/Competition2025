#include "commands/SetLeds.h"
#include "commands/ChassisDriveTime.h"

#include "commands/AutonomousParallel.h"

AutonomousParallel::AutonomousParallel(Leds *leds, Drivetrain *drivetrain)
{
    // Set the command name
    SetName("AutonomousParallel");

    AddCommands(ChassisDriveTime(2_s, 0.5_mps, drivetrain), SetLeds(LedMode::Rainbow, 5_s, leds));
}
