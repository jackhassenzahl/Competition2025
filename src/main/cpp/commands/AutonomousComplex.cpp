#include "commands/AutonomousComplex.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
AutonomousComplex::AutonomousComplex(Leds *leds, Drivetrain *drivetrain)
{
  // Add your commands here, e.g.
  AddCommands(AutonomousLed(leds), AutonomousParallel(leds, drivetrain));
}
