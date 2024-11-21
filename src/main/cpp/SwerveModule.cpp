#include "SwerveModule.h"

#include <cmath>

/// @brief Class constructor for the SwerveModule class.
/// @param driveMotorCANid The CAN ID for the swerve module drive motor.
/// @param angleMotorCANid The CAN ID for the swerve module angle motor.
/// @param angleEncoderCANid The CAN ID for the swerve module angle encoder.
SwerveModule::SwerveModule(int driveMotorCANid, int angleMotorCANid, int angleEncoderCANid)
{
    // Initialize the angle and drive to zero
    m_wheelVector.Angle = 0.0;
    m_wheelVector.Drive = 0.0;
}

/// @brief Set the swerve module angle and motor power.
/// @param vector The wheel vector (angle and drive).
void SwerveModule::SetState(WheelVector vector)
{
    // Optimize the serve module vector to minimize wheel rotation on change of diretion
    OptimizeWheelAngle(vector, &m_wheelVector);

    // Set the Drive motor power

    // Set the angle motor PID set angle
}

/// @brief Method to determine the optimal swerve module wheel angle given the desired wheel vector.
/// @brief Note: The swerve module angle is not restricted to -180 to 180 degrees, but is the actual module angle.
/// @param wheelVector The target swerve module wheel drive power and angle.
void SwerveModule::OptimizeWheelAngle(WheelVector targetWheelVector, WheelVector *wheelVector)
{
    double driveDirection = 1.0;  // Forward direction

    // Convert the present wheel angle to the same hemi-sphere as the target wheel angle
    double workingAngle = ConvertAngleToTargetRange(*wheelVector);
                                                                               
    // Determine the angle between the past and desired swerve angle
    double angleDifference = targetWheelVector.Angle - workingAngle;

    // Determine if the angle is greater that obtuse (greater that 180 degrees)
    if (angleDifference > 180.0)
    {
        // Get the accute angle and change wheel correction direction
        angleDifference = angleDifference - 180.0;
        driveDirection *= -1.0;
    }
    else if (angleDifference < -180.0)
    {
        // Get the accute angle and change wheel correction direction
        angleDifference = angleDifference + 180.0;
        driveDirection *= -1.0;
    }

    // Minimize the wheel rotation
    if (angleDifference > 90.0)
    {
        // Get the minimized wheel angle and change wheel correction direction
        angleDifference = angleDifference - 180.0;

        // Reverse the drive direction
        driveDirection *= -1.0;
    }
    else if (angleDifference < -90.0)
    {
        // Get the minimized wheel angle and change wheel correction direction
        angleDifference = angleDifference + 180.0;

        // Reverse the drive direction
        driveDirection *= -1.0;
    }

    // Set the wheel vector to the target
    wheelVector->Angle += angleDifference;
    wheelVector->Drive  = targetWheelVector.Drive * driveDirection;
}

/// <summary>
/// Convert any angle to the range -180 to 180 degrees.
/// </summary>
/// <param name="angle">The angle to convert.</param>
/// <returns>The angle represented from -180 to 180 degrees.</returns>
double SwerveModule::ConvertAngleToTargetRange(WheelVector wheelVector)
{
    // Get the angle between -360 and 360
    double angle = remainder(wheelVector.Angle, 360.0);
                  
    // Convert large negative angles
    if (angle <= -180.0)
        angle += 360.0;

    // Convert large postive angles
    if (angle > 180.0)
        angle -= 360.0;

    // Return the swerve angle in the proper hemisphere (-180 to 180 degrees)
    return angle;
}

/// <summary>
/// Method to get the swerve module wheel vector.
/// </summary>
/// <param name="wheelVector">Variable to return the swerve module wheel vector.</param>
void SwerveModule::GetWheelVector(WheelVector* wheelVector)
{
    // Return the wheel vector
    *wheelVector = m_wheelVector;
}
