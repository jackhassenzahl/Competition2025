#include "commands/ChassisTurnAngle.h"

ChassisTurnAngle::ChassisTurnAngle(units::angle::degrees angle, double speed, units::time::second_t timeoutTime, Drivetrain *drivetrain) : 
                                   m_angle(angle), m_timeoutTime(timeoutTime), m_speed(speed), m_drivetrain(drivetrain)
{
  // Use addRequirements() here to declare subsystem dependencies

}

/// @brief Called just before this Command runs the first time.
void ChassisTurnAngle::Initialize()
{

}

/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisTurnAngle::Execute() 
{

}

/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool ChassisTurnAngle::IsFinished()
{
  return false;
}

/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void ChassisTurnAngle::End(bool interrupted) 
{

}
