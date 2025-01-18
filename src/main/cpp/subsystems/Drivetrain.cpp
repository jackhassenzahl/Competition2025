#include "subsystems/DriveTrain.h"

/// @brief Field centric, so use gyro.
/// @param forward The forward operater input.
/// @param strafe The strafe operater input.
/// @param rotation The rotation angle operater input.
/// @param gyro The robot direction in relation to the field.
void Drivetrain::Drive(units::meters_per_second_t xSpeed,
                       units::meters_per_second_t ySpeed,
                       units::radians_per_second_t rotation,
                       units::second_t period)
{
    auto states = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
                               m_fieldCentricity ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                               xSpeed, ySpeed, rotation, m_gyro.GetRotation2d()) : frc::ChassisSpeeds{xSpeed, ySpeed, rotation}, period));

    m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

    auto [frontLeft, frontRight, rearLeft, rearRight] = states;

    frc::SmartDashboard::PutNumber("Front Right Angle", frontRight.angle.Degrees().to<double>());
    frc::SmartDashboard::PutNumber("Front Right Drive", frontRight.speed.value());

    frc::SmartDashboard::PutNumber("Front Left Angle",  frontLeft.angle.Degrees().to<double>());
    frc::SmartDashboard::PutNumber("Front Left Drive",  frontLeft.speed.value());

    frc::SmartDashboard::PutNumber("Rear Right Angle",  rearLeft.angle.Degrees().to<double>());
    frc::SmartDashboard::PutNumber("Rear Right Drive",  rearLeft.speed.value());

    frc::SmartDashboard::PutNumber("Rear Left Angle",   rearRight.angle.Degrees().to<double>());
    frc::SmartDashboard::PutNumber("Rear Left Drive",   rearRight.speed.value());

    m_frontLeft.SetState(frontLeft);
    m_frontRight.SetState(frontRight);
    m_backLeft.SetState(rearLeft);
    m_backRight.SetState(rearRight);
}

void Drivetrain::UpdateOdometry()
{
    m_odometry.Update(m_gyro.GetRotation2d(), {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backLeft.GetPosition(), m_backRight.GetPosition()});
}

/// @brief Method to set the robot control field centricity.
/// @param fieldCentric Boolean to indicate if the robor control should be field centric.
void Drivetrain::SetFieldCentricity(bool fieldCentric)
{
    // Set the field centric member variable
    m_fieldCentricity = fieldCentric;
}

/// @brief  
/// @return 
bool Drivetrain::GetFieldCentricity()
{
    // Return the field centricity setting
    return m_fieldCentricity;
}
