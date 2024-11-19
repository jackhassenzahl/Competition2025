#include "Constants.h"
#include "subsystems/Drivetrain.h"

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
    m_swerveModule[0] = new SwerveModule(SWERVE_FRONT_RIGHT_DRIVE_MOTOR_CAN_ID, SWERVE_FRONT_RIGHT_ANGLE_MOTOR_CAN_ID, SWERVE_FRONT_RIGHT_ANGLE_ENCODER_CAN_ID);
    m_swerveModule[1] = new SwerveModule(SWERVE_FRONT_LEFT_DRIVE_MOTOR_CAN_ID,  SWERVE_FRONT_LEFT_ANGLE_MOTOR_CAN_ID,  SWERVE_FRONT_LEFT_ANGLE_ENCODER_CAN_ID);
    m_swerveModule[2] = new SwerveModule(SWERVE_REAR_LEFT_DRIVE_MOTOR_CAN_ID,   SWERVE_REAR_LEFT_ANGLE_MOTOR_CAN_ID,   SWERVE_REAR_LEFT_ANGLE_ENCODER_CAN_ID);
    m_swerveModule[3] = new SwerveModule(SWERVE_REAR_RIGHT_DRIVE_MOTOR_CAN_ID,  SWERVE_REAR_RIGHT_ANGLE_MOTOR_CAN_ID,  SWERVE_REAR_RIGHT_ANGLE_ENCODER_CAN_ID);
}

/// @brief Robot centric, therefore no need for gyro.
/// @param forward The forward operater input.
/// @param strafe The strafe operater input.
/// @param angle The angle operater input.
void Drivetrain::Drive(double forward, double strafe, double angle)
{
    WheelVector wheelVector[NUMBER_OF_SWERVE_MODULES];     // Used for wheel vector calculations

    // Calcualte the drive paramters
    CalculateSwerveModuleDriveAndAngle(forward, strafe, angle, wheelVector);

    // Update the swerve module
    for (int swerveModuleIndex = 0; swerveModuleIndex < NUMBER_OF_SWERVE_MODULES; swerveModuleIndex++)
        m_swerveModule[swerveModuleIndex]->SetState(wheelVector[swerveModuleIndex]);
}

/// @brief Field centric, so use gyro.
/// @param forward The forward operater input.
/// @param strafe The strafe operater input.
/// @param angle The angle operater input.
/// @param gyro The robot direction in relation to the field.
void Drivetrain::Drive(double forward, double strafe, double angle, double gyro)
{
    // Convert to field centric
    FieldCentricAngleConversion(&forward, &strafe, angle);

    // Calcualte the drive paramters
    Drive(forward, strafe, angle);
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
    double A = strafe  - rotate * (CHASSIS_LENGTH / R);
    double B = strafe  + rotate * (CHASSIS_LENGTH / R);
    double C = forward - rotate * (CHASSIS_WIDTH  / R);
    double D = forward + rotate * (CHASSIS_WIDTH  / R);

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
    for (int wheelVectorIndex = 1; wheelVectorIndex < NUMBER_OF_SWERVE_MODULES; wheelVectorIndex++)
        if (wheelVector[wheelVectorIndex].Drive > maxSpeed)
            maxSpeed = wheelVector[wheelVectorIndex].Drive;

    // Normalizes speeds so they're within the ranges of -1 to 1
    if (maxSpeed > 1)
        for (int wheelVectorIndex = 0; wheelVectorIndex < NUMBER_OF_SWERVE_MODULES; wheelVectorIndex++)
            wheelVector[wheelVectorIndex].Drive /= maxSpeed;
}
