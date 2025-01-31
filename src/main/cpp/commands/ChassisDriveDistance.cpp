#include "commands/ChassisDriveDistance.h"

#pragma region ChassisDriveDistance (constructor)
/// @brief Command to drive the robot the specified distance.
/// @param distance The distance to drive the robot.
/// @param speed The speed to perform the drive.
/// @param drivetrain The Drivetrains subsystem.
ChassisDriveDistance::ChassisDriveDistance(units::meter_t distance, units::meters_per_second_t speed, units::time::second_t timeoutTime, Drivetrain *drivetrain) :
                                           m_distance(distance), m_speed(speed), m_timeoutTime(timeoutTime), m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDriveDistance");

    // Declare subsystem dependencies
    AddRequirements(drivetrain);
}
#pragma endregion

#pragma region Initialize
/// @brief Called just before this Command runs the first time.
void ChassisDriveDistance::Initialize()
{
    // Remember the field centric setting
    m_fieldCentricity = m_drivetrain->GetFieldCentricity();

    // Do not use field coordinates
    m_drivetrain->SetFieldCentricity(false);

    // Get the start time
    m_startTime = frc::GetTime();
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisDriveDistance::Execute()
{
    // Start driving
    m_drivetrain->Drive(m_speed, 0_mps, 0_rad_per_s);
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool ChassisDriveDistance::IsFinished()
{
    // Determine if the time-out time has expired
    if (frc::GetTime() - m_startTime > m_timeoutTime)
        return true;

   // Still driving
    return false;
}
#pragma endregion

#pragma region End
/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void ChassisDriveDistance::End(bool interrupted)
{
    // Stop the move
    m_drivetrain->Drive(0_mps, 0_mps, 0_rad_per_s);

    // Restore the field centricity
    m_drivetrain->SetFieldCentricity(m_fieldCentricity);
}
#pragma endregion
