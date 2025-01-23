#include "subsystems/DriveTrain.h"

#pragma region Periodic
/// @brief This method will be called once periodically.
void Drivetrain::Periodic()
{
    frc::SmartDashboard::PutNumber("Gyro Angle",        GetHeading().value());
    frc::SmartDashboard::PutBoolean("Field Centricity", m_fieldCentricity);
}
#pragma endregion

#pragma region Drive
/// @brief Method to drive the robot chassis.
/// @param xSpeed The speed in the X dirction.
/// @param ySpeed The speed in the Y dirction
/// @param rotation The rate of rotation.
/// @param period The robot update period.
void Drivetrain::Drive(units::meters_per_second_t  xSpeed,
                       units::meters_per_second_t  ySpeed,
                       units::radians_per_second_t rotation,
                       units::second_t period)
{
    frc::SmartDashboard::PutNumber("Chassis Forward", (double) xSpeed);
    frc::SmartDashboard::PutNumber("Chassis Strafe",  (double) ySpeed);
    frc::SmartDashboard::PutNumber("Chassis Angle",   (double) rotation);

    auto states = m_kinematics.ToSwerveModuleStates(frc::ChassisSpeeds::Discretize(
                               m_fieldCentricity ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                               xSpeed, ySpeed, rotation, m_gyro.GetRotation2d()) : frc::ChassisSpeeds{xSpeed, ySpeed, rotation}, period));

    m_kinematics.DesaturateWheelSpeeds(&states, kMaxSpeed);

    auto [frontLeft, frontRight, rearLeft, rearRight] = states;

    frc::SmartDashboard::PutNumber("Front Right Drive", frontRight.speed.value());
    frc::SmartDashboard::PutNumber("Front Right Angle", frontRight.angle.Degrees().to<double>());

    frc::SmartDashboard::PutNumber("Front Left Drive",  frontLeft.speed.value());
    frc::SmartDashboard::PutNumber("Front Left Angle",  frontLeft.angle.Degrees().to<double>());

    frc::SmartDashboard::PutNumber("Rear Right Drive",  rearLeft.speed.value());
    frc::SmartDashboard::PutNumber("Rear Right Angle",  rearLeft.angle.Degrees().to<double>());

    frc::SmartDashboard::PutNumber("Rear Left Drive",   rearRight.speed.value());
    frc::SmartDashboard::PutNumber("Rear Left Angle",   rearRight.angle.Degrees().to<double>());

    m_frontLeft.SetState(frontLeft);
    m_frontRight.SetState(frontRight);
    m_backLeft.SetState(rearLeft);
    m_backRight.SetState(rearRight);

    // Read the swerve module angles and drive
    // frc::SmartDashboard::PutNumber("Vector Front Right Drive", m_swerveModule[0]->GetWheelVector()->Drive);
    // frc::SmartDashboard::PutNumber("Vector Front Right Angle", m_swerveModule[0]->GetWheelVector()->Angle);
    // frc::SmartDashboard::PutNumber("Vector Front Left Drive",  m_swerveModule[1]->GetWheelVector()->Drive);
    // frc::SmartDashboard::PutNumber("Vector Front Left Angle",  m_swerveModule[1]->GetWheelVector()->Angle);
    // frc::SmartDashboard::PutNumber("Vector Rear Left Drive",   m_swerveModule[2]->GetWheelVector()->Drive);
    // frc::SmartDashboard::PutNumber("Vector Rear Left Angle",   m_swerveModule[2]->GetWheelVector()->Angle);
    // frc::SmartDashboard::PutNumber("Vector Rear Right Drive",  m_swerveModule[3]->GetWheelVector()->Drive);
    // frc::SmartDashboard::PutNumber("Vector Rear Right Angle",  m_swerveModule[3]->GetWheelVector()->Angle);
}
#pragma endregion

#pragma region UpdateOdometry
/// @brief Method to update the robot odometry.
void Drivetrain::UpdateOdometry()
{
    m_odometry.Update(m_gyro.GetRotation2d(), {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_backLeft.GetPosition(), m_backRight.GetPosition()});
}
#pragma endregion

#pragma region SetFieldCentricity
/// @brief Method to set the robot control field centricity.
/// @param fieldCentric Boolean to indicate if the robor control should be field centric.
void Drivetrain::SetFieldCentricity(bool fieldCentric)
{
    // Set the field centric member variable
    m_fieldCentricity = fieldCentric;
}
#pragma endregion

#pragma region GetFieldCentricity
/// @brief Method to set the field centricity.
/// @return The field centricity setting.
bool Drivetrain::GetFieldCentricity()
{
    // Return the field centricity setting
    return m_fieldCentricity;
}
#pragma endregion

#pragma region SetSwerveWheelAnglesToZero
/// @brief Method to set the swerve wheel to the absoulute encoder angle then zero the PID controller angle.
void Drivetrain::SetSwerveWheelAnglesToZero()
{
    // Set the swerve wheel angles to zero
    // for (auto swerveModuleIndex = 0; swerveModuleIndex < ChassisConstants::NumberOfSwerveModules; swerveModuleIndex++)  TODO:
    //     m_swerveModule[swerveModuleIndex]->SetSwerveWheelAnglesToZero();
}
#pragma endregion

#pragma region GetHeading
/// @brief Method to get the robot heading.
/// @return The robot heading.
units::degree_t Drivetrain::GetHeading()
{
    // Return the robot heading
    return (units::degree_t) m_gyro.GetAngle();
}
#pragma endregion
