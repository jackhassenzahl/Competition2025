#include "commands/ChassisDriveToAprilTag.h"

#pragma region
/// @brief Command to drive the chassis to an AprilTag.
/// @param speed The speed to move the chassis.
/// @param timeoutTime The timeout time for the rotation.
/// @param aprilTags The AprilTag subsystem.
/// @param drivetrain The Drivetrain subsystem.
ChassisDriveToAprilTag::ChassisDriveToAprilTag(units::meters_per_second_t speed, units::time::second_t timeoutTime, AprilTags *aprilTags, Drivetrain *drivetrain) :
                                               m_speed(speed), m_timeoutTime(timeoutTime), m_aprilTags(aprilTags), m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDriveToAprilTag");

    // Declare subsystem dependencies
    AddRequirements(drivetrain);
}
#pragma endregion

#pragma region Initialize
/// @brief Called just before this Command runs the first time.
void ChassisDriveToAprilTag::Initialize()
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
void ChassisDriveToAprilTag::Execute()
{
    AprilTagInformation aprilTagInformation;

    if (m_aprilTags->GetClosestTag(aprilTagInformation) == true)
    {
        // Use the retrieved values (e.g., display them on the SmartDashboard)
        frc::SmartDashboard::PutNumber("Pose X",     aprilTagInformation.X);
        frc::SmartDashboard::PutNumber("Pose Y",     aprilTagInformation.Y);
        frc::SmartDashboard::PutNumber("Pose Z",     aprilTagInformation.Z);
        frc::SmartDashboard::PutNumber("Rotation X", aprilTagInformation.rotationX);
        frc::SmartDashboard::PutNumber("Rotation Y", aprilTagInformation.rotationY);
        frc::SmartDashboard::PutNumber("Rotation Z", aprilTagInformation.rotationZ);
    }
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool ChassisDriveToAprilTag::IsFinished()
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
void ChassisDriveToAprilTag::End(bool interrupted)
{
    // Stop the move
    m_drivetrain->Drive(0_mps, 0_mps, 0_rad_per_s);

    // Restore the field centricity
    m_drivetrain->SetFieldCentricity(m_fieldCentricity);
}
#pragma endregion