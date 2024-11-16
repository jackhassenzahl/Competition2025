#include "commands/DriveDistance.h"

DriveDistance::DriveDistance(double Distance, Drivetrain *m_drivetrain) : m_Distance(Distance), m_drivetrain(m_drivetrain)
{
    // Set the command name
    SetName("DriveDistance");

    // Declare subsystem dependencies
    AddRequirements({m_drivetrain});
}

/// @brief Called just before this Command runs the first time.
void DriveDistance::Initialize()
{

}

/// @brief Called repeatedly when this Command is scheduled to run.
void DriveDistance::Execute()
{

}

/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool DriveDistance::IsFinished()
{
    return false;
}

/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void DriveDistance::End(bool interrupted)
{

}

/// @brief Indicates if the command runs when the robot is disabled.
/// @return True is the command should run when the robot is disabled.
bool DriveDistance::RunsWhenDisabled() const
{
    return false;
}
