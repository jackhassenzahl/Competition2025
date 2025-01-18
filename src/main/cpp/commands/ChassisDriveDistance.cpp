#include "commands/ChassisDriveDistance.h"

ChassisDriveDistance::ChassisDriveDistance(units::meter_t distance, units::meters_per_second_t speed, Drivetrain *drivetrain) : m_distance(distance), m_speed(speed), m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDriveDistance");

    // Declare subsystem dependencies
    AddRequirements(drivetrain);
}

/// @brief Called just before this Command runs the first time.
void ChassisDriveDistance::Initialize()
{
    // Remember the field centric setting
    m_fieldCentricity = m_drivetrain->GetFieldCentricity();
    
    // Do not use field coordinates
    m_drivetrain->SetFieldCentricity(false);
}

/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisDriveDistance::Execute()
{
    // Start driving
    m_drivetrain->Drive(m_speed, 0_mps, 0_rad_per_s, RobotContainer::GetInstance()->GetPeriod());
}

/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool ChassisDriveDistance::IsFinished()
{
    // Determine if the chassis has completed the drive distance
    return true;
}

/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void ChassisDriveDistance::End(bool interrupted)
{
    // Stop the move
    m_drivetrain->Drive(0_mps, 0_mps, 0_rad_per_s, RobotContainer::GetInstance()->GetPeriod());

    // Restore the field centricity
    m_drivetrain->SetFieldCentricity(m_fieldCentricity);
}
