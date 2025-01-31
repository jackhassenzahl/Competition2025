#include "commands/ChassisDriveTurnAngle.h"

#pragma region ChassisDriveTurnAngle (constructor)
/// @brief Command to rotate the chassis to the specified angle.
/// @param angle The desire robot angle.
/// @param speed The speed to move the chassis.
/// @param timeoutTime The timeout time for the rotation.
/// @param drivetrain The Drivetrain subsystem.
ChassisDriveTurnAngle::ChassisDriveTurnAngle(units::angle::degree_t angle, units::meters_per_second_t speed, units::time::second_t timeoutTime, Drivetrain *drivetrain) :
                                   m_angle(angle), m_speed(speed), m_timeoutTime(timeoutTime), m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDriveTurnAngle");

    // Declare subsystem dependencies
    AddRequirements(drivetrain);
}
#pragma endregion

#pragma region Initialize
/// @brief Called just before this Command runs the first time.
void ChassisDriveTurnAngle::Initialize()
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
void ChassisDriveTurnAngle::Execute()
{

}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool ChassisDriveTurnAngle::IsFinished()
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
void ChassisDriveTurnAngle::End(bool interrupted)
{
    // Stop the move
    m_drivetrain->Drive(0_mps, 0_mps, 0_rad_per_s);

    // Restore the field centricity
    m_drivetrain->SetFieldCentricity(m_fieldCentricity);
}
#pragma endregion
