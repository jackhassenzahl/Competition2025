#include "commands/AutonomousDoNothing.h"

/// @brief Autonomous command to do nothing.
AutonomousDoNothing::AutonomousDoNothing()
{
    // Set the command name
    SetName("AutonomousDoNothing");
}

/// @brief Called just before this Command runs the first time.
void AutonomousDoNothing::Initialize()
{

}

/// @brief Called repeatedly when this Command is scheduled to run.
void AutonomousDoNothing::Execute()
{

}

/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool AutonomousDoNothing::IsFinished()
{
    return false;
}

/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void AutonomousDoNothing::End(bool interrupted)
{

}

/// @brief Indicates if the command runs when the robot is disabled.
/// @return True is the command should run when the robot is disabled.
bool AutonomousDoNothing::RunsWhenDisabled() const
{
    return false;
}
