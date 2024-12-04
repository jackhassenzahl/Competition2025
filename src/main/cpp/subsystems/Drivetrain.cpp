#include "Constants.h"
#include "subsystems/DriveTrain.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <iostream>

/// @brief Class constructor for the DriveTrain subassembly.
/// Swerve Module Indexes:
///
///          Front
///       +---------+ ---
///       |[1]   [0]|  ^       0   Front Right
///       |         |  |       1   Front Left
///       |         | Length   2   Rear Left
///       |         |  |       3   Rear Right
///       |[2]   [3]|  v
///       +---------+ ---
///       |         |
///       |< Width >|
Drivetrain::Drivetrain()
{
    // Create the robot swerve modules
    m_swerveModule[0] = new SwerveModule(CanConstants::kSwerveFrontRightDriveMotorCanId,
                                         CanConstants::kSwerveFrontRightAngleMotorCanId,
                                         CanConstants::kSwerveFrontRightAngleEncoderCanId);
    m_swerveModule[1] = new SwerveModule(CanConstants::kSwerveFrontLeftDriveMotorCanId,
                                         CanConstants::kSwerveFrontLeftAngleMotorCanId,
                                         CanConstants::kSwerveFrontLeftAngleEncoderCanId);
    m_swerveModule[2] = new SwerveModule(CanConstants::kSwerveRearLeftDriveMotorCanId,
                                         CanConstants::kSwerveRearLeftAngleMotorCanId,
                                         CanConstants::kSwerveRearLeftAngleEncoderCanId);
    m_swerveModule[3] = new SwerveModule(CanConstants::kSwerveRearRightDriveMotorCanId,
                                         CanConstants::kSwerveRearRightAngleMotorCanId,
                                         CanConstants::kSwerveRearRightAngleEncoderCanId);
}

/// @brief Field centric, so use gyro.
/// @param forward The forward operater input.
/// @param strafe The strafe operater input.
/// @param angle The angle operater input.
/// @param gyro The robot direction in relation to the field.
void Drivetrain::Drive(double forward, double strafe, double angle, double gyro)
{
    frc::SmartDashboard::PutNumber("Chassis Forward", forward);
    frc::SmartDashboard::PutNumber("Chassis Strafe",  strafe);
    frc::SmartDashboard::PutNumber("Chassis Angle",   angle);
    frc::SmartDashboard::PutNumber("Chassis Gyro",    gyro);

    // Convert to field centric
    if (m_fieldCentricity)
       FieldCentricAngleConversion(&forward, &strafe, angle);

    // Create a wheel vector array for wheel vector calculations
    WheelVector wheelVector[ChassisConstants::kNumberOfSwerveModules];    

    // Calcualte the drive paramters
    CalculateSwerveModuleDriveAndAngle(forward, strafe, angle, wheelVector);

    ///          Front
    ///       +---------+ ---
    ///       |[1]   [0]|  ^       0   Front Right
    ///       |         |  |       1   Front Left
    ///       |         | Length   2   Rear Left
    ///       |         |  |       3   Rear Right
    ///       |[2]   [3]|  v
    ///       +---------+ ---
    ///       |         |
    ///       |< Width >|

    frc::SmartDashboard::PutNumber("Front Right Drive", wheelVector[0].Drive);
    frc::SmartDashboard::PutNumber("Front Right Angle", wheelVector[0].Angle);
    frc::SmartDashboard::PutNumber("Front Left Drive",  wheelVector[1].Drive);
    frc::SmartDashboard::PutNumber("Front Left Angle",  wheelVector[1].Angle);
    frc::SmartDashboard::PutNumber("Rear Left Drive",   wheelVector[2].Drive);
    frc::SmartDashboard::PutNumber("Rear Left Angle",   wheelVector[2].Angle);
    frc::SmartDashboard::PutNumber("Rear Right Drive",  wheelVector[3].Drive);
    frc::SmartDashboard::PutNumber("Rear Right Angle",  wheelVector[3].Angle);

    // Update the swerve module
    for (auto swerveModuleIndex = 0; swerveModuleIndex < ChassisConstants::kNumberOfSwerveModules; swerveModuleIndex++)
        m_swerveModule[swerveModuleIndex]->SetState(wheelVector[swerveModuleIndex]);
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

/// <summary>
/// Method to get the specified swerve module wheel vector.
/// </summary>
/// <param name="swerveModuleIndex">The swerve module index.</param>
/// <param name="wheelVector">Variable to return the specified swerve module wheel vector.</param>
void Drivetrain::GetSwerveModuleWheelVector(int swerveModuleIndex, WheelVector* wheelVector)
{
    // Get the specified swerve module wheel vector
    m_swerveModule[swerveModuleIndex]->GetWheelVector(wheelVector);
}

/// <summary>
/// Methiod to convert the forward and strafe into field centric values based on the gyro angle.
///
/// Note: The drive motor range is 0.0 to 1.0 and the angle is in the range -180 to 180 degrees.
/// </summary>
/// <param name="forward">The forward power.</param>
/// <param name="strafe">The strafe (side) power.</param>
/// <param name="angle">The present robot angle relative to the field direction.</param>
void Drivetrain::FieldCentricAngleConversion(double *forward, double *strafe, double angle)
{
    // Copy the forward and strafe method parameters
    double forwardParameter = *forward;
    double strafeParamewter = *strafe;

    // Convert the angle from degrees to radians
    angle = angle * PI / 180;

    // Modify the input parameters for field centric control
    *forward =  forwardParameter * cos(angle) + strafeParamewter * sin(angle);
    *strafe  = -forwardParameter * sin(angle) + strafeParamewter * cos(angle);
}

/// <summary>
/// Method to output an array of speed and rotation values for each swerve module for a drive train given
/// the desired forward, strafe, and rotation.
///
/// See: "Derivation of Inverse Kinematics for Swerve.pdf" for calcuation details located at
///      https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
///
/// Swerve Module Indexes:
///
///          Front
///       +---------+ ---
///       |[1]   [0]|  ^       0   Front Right
///       |         |  |       1   Front Left
///       |         | Length   2   Rear Left
///       |         |  |       3   Rear Right
///       |[2]   [3]|  v
///       +---------+ ---
///       |         |
///       |< Width >|
///
/// </summary>
/// <param name="forward">positive value = forward movement,   negative value = backward movement</param>
/// <param name="strafe">positive value  = right direction,    negative value = left direction</param>
/// <param name="rotate">positive value  = clockwise rotation, negative value = counterclockwise rotation</param>
void Drivetrain::CalculateSwerveModuleDriveAndAngle(double forward, double strafe, double rotate, WheelVector wheelVector[])
{
    // Create intermediate values for the speed and angle calculations
    double A = strafe  - rotate * (ChassisConstants::kChassisLength / R);
    double B = strafe  + rotate * (ChassisConstants::kChassisLength / R);
    double C = forward - rotate * (ChassisConstants::kChassisWidth  / R);
    double D = forward + rotate * (ChassisConstants::kChassisWidth  / R);

    // Calculate the wheel angle and convert radians to degrees
    wheelVector[0].Angle = atan2(B, C) * 180 / PI;
    wheelVector[1].Angle = atan2(B, D) * 180 / PI;
    wheelVector[2].Angle = atan2(A, D) * 180 / PI;
    wheelVector[3].Angle = atan2(A, C) * 180 / PI;

    // Calculate the speed
    wheelVector[0].Drive = sqrt(B * B + C * C);
    wheelVector[1].Drive = sqrt(B * B + D * D);
    wheelVector[2].Drive = sqrt(A * A + D * D);
    wheelVector[3].Drive = sqrt(A * A + C * C);

    // Normalize the speed values
    NormalizeSpeed(wheelVector);
}

/// <summary>
/// Method to normalize the Drive values for a Swerve Module
/// </summary>
/// <param name="swerveModule">Structure for returning the swerve module normalization for the drive motors.</param>
void Drivetrain::NormalizeSpeed(WheelVector wheelVector[])
{
    // Determine the maximum speed
    double maxSpeed = wheelVector[0].Drive;
    for (auto wheelVectorIndex = 1; wheelVectorIndex < ChassisConstants::kNumberOfSwerveModules; wheelVectorIndex++)
        if (wheelVector[wheelVectorIndex].Drive > maxSpeed)
            maxSpeed = wheelVector[wheelVectorIndex].Drive;

    // Normalizes speeds so they're within the ranges of -1 to 1
    if (maxSpeed > 1)
        for (auto wheelVectorIndex = 0; wheelVectorIndex < ChassisConstants::kNumberOfSwerveModules; wheelVectorIndex++)
            wheelVector[wheelVectorIndex].Drive /= maxSpeed;
}
