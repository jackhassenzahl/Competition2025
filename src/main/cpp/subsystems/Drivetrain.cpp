#include "subsystems/Drivetrain.h"

#include <iostream>

/// @brief 
Drivetrain::Drivetrain()
{

}

// Robot centric, therefore no need for gyro
void Drivetrain::Drive(double forward, double strafe, double angle)
{
    CalculateSwerveModuleDriveAndAngle(forward, strafe, angle, &m_mathSwerveModule);
}

// Field centric, so use gyro
void Drivetrain::Drive(double forward, double strafe, double angle, double gyro)
{

}

/// <summary>
/// Methiod to convert the forward and strafe into field centric values based on the gyro angle.
///
/// Note: The drive motor range is 0.0 to 1.0 and the angle is in the range -180 to 180 degrees.
/// </summary>
/// <param name="forward"></param>
/// <param name="strafe"></param>
/// <param name="angle"></param>
/// <param name="swerveModule"></param>
void Drivetrain::FieldCentricAngleConversion(double *forward, double *strafe, double angle)
{
    // Copy the forward and strafe method parameters
    double forwardParameter = *forward;
    double strafeParamewter = *strafe;

    // Convert the angle from degrees to radians
    angle = angle * PI / 180;

    // Modify the input parameters for field centric control
    *forward = forwardParameter * cos(angle) + strafeParamewter * sin(angle);
    *strafe = -forwardParameter * sin(angle) + strafeParamewter * cos(angle);
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
/// <param name="swerveModule">Structure for returning the swerve module calculations for drive and angle motors.</param>
void Drivetrain::CalculateSwerveModuleDriveAndAngle(double forward, double strafe, double rotate, MathSwerveModule *swerveModule)
{
    // Create intermediate values for the speed and angle calculations
    double A = strafe  - rotate * (ChassisLength / R);
    double B = strafe  + rotate * (ChassisLength / R);
    double C = forward - rotate * (ChassisWidth  / R);
    double D = forward + rotate * (ChassisWidth  / R);

    // Calculate the wheel angle and convert radians to degrees
    swerveModule->Angle[0] = atan2(B, C) * 180 / PI;
    swerveModule->Angle[1] = atan2(B, D) * 180 / PI;
    swerveModule->Angle[2] = atan2(A, D) * 180 / PI;
    swerveModule->Angle[3] = atan2(A, C) * 180 / PI;

    // Calculate the speed
    swerveModule->Drive[0] = sqrt(B * B + C * C);
    swerveModule->Drive[1] = sqrt(B * B + D * D);
    swerveModule->Drive[2] = sqrt(A * A + D * D);
    swerveModule->Drive[3] = sqrt(A * A + C * C);

    // Normalize the speed values
    NormalizeSpeed(swerveModule);
}

/// <summary>
/// Method to determine the optimal swerve module angle given the present angle and the desired drive vector.
///
/// Note: The past angle is not restricted to -180 to 180 degrees, but is the actual module angle.
/// </summary>
/// <param name="pastSwerveModule">The past swerve module drive power and angle.</param>
/// <param name="desiredSwerveModule">The desired swerve module drive power and angle.</param>
/// <param name="newSwerveModule">The optimized swerve module drive power and angle.</param>
void Drivetrain::OptimizeWheelAngle(MathSwerveModule pastSwerveModule, MathSwerveModule desiredSwerveModule, MathSwerveModule *newSwerveModule)
{
    // Initialize the new swerve module to the desired in case no changes are needed
    for (int swerveModule = 0; swerveModule < NumberOfSwerveModules; swerveModule++)
    {
        newSwerveModule->Drive[swerveModule] = desiredSwerveModule.Drive[swerveModule];
        newSwerveModule->Angle[swerveModule] = desiredSwerveModule.Angle[swerveModule];
    }

    // Loop through all swerve modules
    for (int swerveModule = 0; swerveModule < NumberOfSwerveModules; swerveModule++)
    {
        double direction = 1.0; // Assume the past swerve power is positive

        // Determine if the past swerve module power is negative
        if (pastSwerveModule.Drive[swerveModule] < 0.0)
            direction = -1.0;

        // Determine the minimum angle between the past and desired swerve angle
        double angleDifference = desiredSwerveModule.Angle[swerveModule] - pastSwerveModule.Angle[swerveModule];

        // printf("SwerveModule: %8.2f  %8.2f  %8.2f  ", pastSwerveModule[swerveModule][1], desiredSwerveModule[swerveModule][1], angleDifference);

        // Determine if the angle is greater than 180 degrees
        if (angleDifference >= 180.0)
        {
            // Invert the drive power and angle
            newSwerveModule->Drive[swerveModule] = desiredSwerveModule.Drive[swerveModule] * -1.0 * direction;
            angleDifference -= 180.0;
        }

        // Determine if the angle is less than 180 degrees
        if (angleDifference <= -180.0)
        {
            // Invert the drive power and angle
            newSwerveModule->Drive[swerveModule] = desiredSwerveModule.Drive[swerveModule] * -1.0 * direction;
            angleDifference += 180.0;
        }

        // Calculate the new swerve module angle
        newSwerveModule->Angle[swerveModule] = pastSwerveModule.Angle[swerveModule] + angleDifference;

        // printf("%8.2f  %8.2f  %8.2f\n", angleDifference, newSwerveModule[swerveModule][1], newSwerveModule[swerveModule][0]);
    }
}

/// <summary>
/// Method to normalize the Drive values for a Swerve Module
/// </summary>
/// <param name="swerveModule">Structure for returning the swerve module normalization for the drive motors.</param>
void Drivetrain::NormalizeSpeed(MathSwerveModule *swerveModule)
{
    // Determine the maximum speed
    double maxSpeed = swerveModule->Drive[0];
    for (int swerveModuleIndex = 1; swerveModuleIndex < NumberOfSwerveModules; swerveModuleIndex++)
        if (swerveModule->Drive[swerveModuleIndex] > maxSpeed)
            maxSpeed = swerveModule->Drive[swerveModuleIndex];

    // Normalizes speeds so they're within the ranges of -1 to 1
    if (maxSpeed > 1)
        for (int swerveModuleIndex = 0; swerveModuleIndex < NumberOfSwerveModules; swerveModuleIndex++)
            swerveModule->Drive[swerveModuleIndex] /= maxSpeed;
}