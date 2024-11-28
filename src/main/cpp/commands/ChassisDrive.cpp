#include "commands/ChassisDrive.h"
#include "subsystems/Drivetrain.h"

/// @brief The operator chassis drive command.
/// @param Left 
/// @param strafe
/// @param m_drivetrain

/// @brief The operator  chassis drive command.
/// @param forward The forward driver input.
/// @param strafe The strafe driver input.
/// @param angle The angle driver input.
/// @param gyro The robot orientation determined by the gyro.
/// @param m_drivetrain The drive train subsystem.
ChassisDrive::ChassisDrive(std::function<double()> forward,
                           std::function<double()> strafe,
                           std::function<double()> angle,
                           std::function<double()> gyro,
                           Drivetrain *drivetrain) :
                           m_forward{std::move(forward)},
                           m_strafe{std::move(strafe)},
                           m_angle{std::move(gyro)},
                           m_gyro{std::move(gyro)},
                           m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDrive");

    // Declare subsystem dependencies
    AddRequirements(drivetrain);
}

/// @brief Called just before this Command runs the first time.
void ChassisDrive::Initialize()
{

}

/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisDrive::Execute()
{
    // Perform the chassis drive
    m_drivetrain->Drive(m_forward(), m_strafe(), m_angle(), m_gyro());
}
