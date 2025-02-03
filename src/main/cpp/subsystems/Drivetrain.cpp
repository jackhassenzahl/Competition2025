#include "subsystems/DriveTrain.h"

using namespace CanConstants;

#pragma region DriveTrain (constructor)
/// @brief The Constructor for the DriveTrain class.
Drivetrain::Drivetrain()
    : m_frontLeft {SwerveFrontLeftDriveMotorCanId,  SwerveFrontLeftAngleMotorCanId,  SwerveFrontLeftAngleEncoderCanId,  ChassisConstants::FrontLeftChassisAngularOffset},
      m_frontRight{SwerveFrontRightDriveMotorCanId, SwerveFrontRightAngleMotorCanId, SwerveFrontRightAngleEncoderCanId, ChassisConstants::FrontRightChassisAngularOffset},
      m_rearLeft  {SwerveRearLeftDriveMotorCanId,   SwerveRearLeftAngleMotorCanId,   SwerveRearLeftAngleEncoderCanId,   ChassisConstants::RearLeftChassisAngularOffset},
      m_rearRight {SwerveRearRightDriveMotorCanId,  SwerveRearRightAngleMotorCanId,  SwerveRearRightAngleEncoderCanId,  ChassisConstants::RearRightChassisAngularOffset},
      m_odometry  {m_kinematics, frc::Rotation2d(units::radian_t{m_gyro.GetAngle()}),
                  {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                   m_rearLeft.GetPosition(),  m_rearRight.GetPosition()}, frc::Pose2d{}}
{
    // Usage reporting for MAXSwerve template
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, HALUsageReporting::kRobotDriveSwerve_MaxSwerve);
}
#pragma endregion

#pragma region Periodic
/// @brief This method will be called once periodically.
void Drivetrain::Periodic()
{
    frc::SmartDashboard::PutNumber("Gyro Angle",           GetHeading().value());
    frc::SmartDashboard::PutBoolean("Field Centricity",    m_fieldCentricity);

    frc::SmartDashboard::PutNumber("Front Right Speed",    m_frontRight.GetState().speed.value());
    frc::SmartDashboard::PutNumber("Front Left Speed",     m_frontLeft.GetState().speed.value());
    frc::SmartDashboard::PutNumber("Rear Right Speed",     m_rearRight.GetState().speed.value());
    frc::SmartDashboard::PutNumber("Rear Left Speed",      m_rearLeft.GetState().speed.value());

    frc::SmartDashboard::PutNumber("Front Right Distance", m_frontRight.GetPosition().distance.value());
    frc::SmartDashboard::PutNumber("Front Left Distance",  m_frontLeft.GetPosition().distance.value());
    frc::SmartDashboard::PutNumber("Rear Right Distance",  m_rearRight.GetPosition().distance.value());
    frc::SmartDashboard::PutNumber("Rear Left Distance",   m_rearLeft.GetPosition().distance.value());

    frc::SmartDashboard::PutNumber("Front Right Angle",    m_frontRight.GetPosition().angle.Degrees().value());
    frc::SmartDashboard::PutNumber("Front Left Angle",     m_frontLeft.GetPosition().angle.Degrees().value());
    frc::SmartDashboard::PutNumber("Rear Right Angle",     m_rearRight.GetPosition().angle.Degrees().value());
    frc::SmartDashboard::PutNumber("Rear Left Angle",      m_rearLeft.GetPosition().angle.Degrees().value());

    // Update the swerve drive odometry
    m_odometry.Update(m_gyro.GetRotation2d(),
                     {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                      m_rearLeft.GetPosition(),  m_rearRight.GetPosition()});
}
#pragma endregion

#pragma region Drive
/// @brief Method to drive the robot chassis.
/// @param xSpeed The speed in the X dirction.
/// @param ySpeed The speed in the Y dirction.
/// @param rotation The rate of rotation.
void Drivetrain::Drive(units::meters_per_second_t  xSpeed,
                       units::meters_per_second_t  ySpeed,
                       units::radians_per_second_t rotation)
{
    // xSpeed   = (units::meters_per_second_t)  frc::SmartDashboard::GetNumber("Chassis Forward", 0.0);
    // ySpeed   = (units::meters_per_second_t)  frc::SmartDashboard::GetNumber("Chassis Strafe",  0.0);
    // rotation = (units::radians_per_second_t) frc::SmartDashboard::GetNumber("Chassis Angle",   0.0);

    frc::SmartDashboard::PutNumber("Chassis Forward", (double) xSpeed);
    frc::SmartDashboard::PutNumber("Chassis Strafe",  (double) ySpeed);
    frc::SmartDashboard::PutNumber("Chassis Angle",   (double) rotation);

    frc::SmartDashboard::PutNumber("Gyro Rotation", (double) m_gyro.GetRotation2d().Degrees());

    auto reverseGyro = m_gyro.GetRotation2d() * -1.0;

    // Determine the swerve module states from teh
    auto states = m_kinematics.ToSwerveModuleStates(m_fieldCentricity ?
                  frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rotation, reverseGyro) :
                  frc::ChassisSpeeds{xSpeed, ySpeed, rotation});

    // Set the module states
    SetModuleStates(states);
}
#pragma endregion

#pragma region SetX
/// @brief
void Drivetrain::SetX()
{
    m_frontLeft. SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{ 45_deg}}, "Front Left " );
    m_frontRight.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}}, "Front Right ");
    m_rearLeft.  SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}}, "Rear Left "  );
    m_rearRight. SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{ 45_deg}}, "Rear Right " );
}
#pragma endregion

#pragma region ResetDriveEncoders
/// @brief Method to reset the drive encoders for each swerve module.
void Drivetrain::ResetDriveEncoders()
{
    // Reset the swerve motor encoders
    m_frontLeft. ResetDriveEncoder();
    m_frontRight.ResetDriveEncoder();
    m_rearLeft.  ResetDriveEncoder();
    m_rearRight. ResetDriveEncoder();
}
#pragma endregion

#pragma region SetModuleStates
/// @brief Method to set the swerve drive states.
/// @param desiredStates
void Drivetrain::SetModuleStates(wpi::array<frc::SwerveModuleState, ChassisConstants::NumberOfSwerveModules> desiredStates)
{
    // Normalize the wheel speeds if any individual speed is above the specified maximum
    m_kinematics.DesaturateWheelSpeeds(&desiredStates, ChassisConstants::MaxSpeed);

    auto [frontLeft, frontRight, rearLeft, rearRight] = desiredStates;

    // Set the swerve module states
    m_frontLeft. SetDesiredState(frontLeft,  "Front Left " );
    m_frontRight.SetDesiredState(frontRight, "Front Right ");
    m_rearLeft.  SetDesiredState(rearLeft,   "Rear Left "  );
    m_rearRight. SetDesiredState(rearRight,  "Rear Right " );
}
#pragma endregion

#pragma region GetHeading
/// @brief Method to get the robot heading.
/// @return The robot heading.
units::radian_t Drivetrain::GetHeading()
{
    // Return the robot heading
    return m_gyro.GetRotation2d().Radians();
}
#pragma endregion

#pragma region ZeroHeading
/// @brief Method to zero the robot heading.
void Drivetrain::ZeroHeading()
{
    // Reset the gyro
    m_gyro.Reset();
}
#pragma endregion

#pragma region GetPose
/// @brief Method to get the pose of the chassis.
/// @return The chassis pose.
frc::Pose2d Drivetrain::GetPose()
{
    // Return the chassis pose
    return m_odometry.GetPose();
}
#pragma endregion

#pragma region ResetOdometry
/// @brief Method to reset the chassis odometry.
/// @param pose The present chassis position on the field.
void Drivetrain::ResetOdometry(frc::Pose2d pose)
{
    // Reset the present odometry
    m_odometry.ResetPosition(GetHeading(), {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                                            m_rearLeft.GetPosition(),  m_rearRight.GetPosition()}, pose);
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

#pragma region SetWheelAnglesToZero
/// @brief Method to set the swerve wheel to the absoulute encoder angle then zero the PID controller angle.
void Drivetrain::SetWheelAnglesToZero()
{
    // Set the swerve wheel angles to zero
    m_frontLeft. SetWheelAngleToForward(SwerveConstants::FrontLeftForwardAngle);
    m_frontRight.SetWheelAngleToForward(SwerveConstants::FrontRightForwardAngle);
    m_rearLeft.  SetWheelAngleToForward(SwerveConstants::RearLeftForwardAngle);
    m_rearRight. SetWheelAngleToForward(SwerveConstants::RearRightForwardAngle);
}
#pragma endregion
