#include "commands/AutonomousOneCoral.h"

AutonomousOneCoral::AutonomousOneCoral()
{
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void AutonomousOneCoral::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutonomousOneCoral::Execute() {}

// Called once the command ends or is interrupted.
void AutonomousOneCoral::End(bool interrupted) {}

// Returns true when the command should end.
bool AutonomousOneCoral::IsFinished()
{
  return false;
}
