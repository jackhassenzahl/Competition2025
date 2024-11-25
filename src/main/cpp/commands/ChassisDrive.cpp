#include "commands/ChassisDrive.h"
#include "subsystems/Drivetrain.h"

/// @brief The operator chassis drive command.
/// @param Left
/// @param Right
/// @param m_drivetrain
ChassisDrive::ChassisDrive(std::function<double()> left, std::function<double()> right, std::function<double()> gyro, Drivetrain *m_drivetrain) : 
                           m_left{std::move(left)}, m_right{std::move(right)}, m_gyro{std::move(gyro)}, m_drivetrain(m_drivetrain)
{
    // Set the command name
    SetName("ChassisDrive");

    // Declare subsystem dependencies
    AddRequirements({m_drivetrain});
}

/// @brief Called just before this Command runs the first time.
void ChassisDrive::Initialize()
{

}

/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisDrive::Execute()
{
    m_drivetrain->Drive(m_left(), m_right(), 0.0, m_gyro());
}

/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool ChassisDrive::IsFinished()
{
    return false;
}

/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void ChassisDrive::End(bool interrupted)
{

}

/// @brief Indicates if the command runs when the robot is disabled.
/// @return True is the command should run when the robot is disabled.
bool ChassisDrive::RunsWhenDisabled() const
{
    return false;
}
