#include "commands/AutonomousComplex.h"

#pragma region AutonomousComplex
/// @brief Constructor for the AutonomousComplex class.
/// @param leds The Leds subsystem.
/// @param drivetrain The Drivetrain subsystem.
AutonomousComplex::AutonomousComplex(Leds *leds, Drivetrain *drivetrain)
{

    // Add commands to the autonomous complex
    AddCommands(AutonomousLed(leds), AutonomousParallel(leds, drivetrain));
}
#pragma endregion
