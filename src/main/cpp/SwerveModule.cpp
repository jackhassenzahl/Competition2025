#include "SwerveModule.h"

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
   // Convert the present wheel angle to the same hemi-sphere as the target wheel angle
   double workingAngle = ConvertAngleToTargetRange(*wheelVector, targetWheelVector);
                                                                               
   // Determine the angle between the past and desired swerve angle
   double angleDifference = targetWheelVector.Angle - workingAngle;

   // Determine if the angle is greater than 180 degrees
   if (angleDifference > 90.0)
   {
      // Invert the drive power and angle
      wheelVector->Drive = targetWheelVector.Drive * -1.0;
      angleDifference -= 180.0;

      // Calculate the new swerve module angle
      wheelVector->Angle += angleDifference;
   }
   else if (angleDifference < -90.0)  // Determine if the angle is less than 180 degrees
   {
      // Invert the drive power and angle
      wheelVector->Drive = targetWheelVector.Drive * -1.0;
      angleDifference += 180.0;

      // Calculate the new swerve module angle
      wheelVector->Angle += angleDifference;
   }
   else
   {
      // Set the wheel vector to the target
      wheelVector->Angle = wheelVector->Angle + angleDifference;
      wheelVector->Drive = targetWheelVector.Drive;
   }
}

/// <summary>
/// Convert any angle to -180 to 180 degrees.
/// </summary>
/// <param name="angle">The angle to convert.</param>
/// <returns>The angle represented from -180 to 180 degrees.</returns>
double SwerveModule::ConvertAngleToTargetRange(WheelVector wheelVector, WheelVector targetWheelVector)
{
    // Determine the target is positive (Compute angle in 0 to 180 degrees range)
    if (targetWheelVector.Angle >= 0.0)
    {
        // If the wheel angle is greater that 180 degrees
        if (wheelVector.Angle > 180.0)
        {
           // Determine if the wheel angle is greater than 360 degrees
           while (wheelVector.Angle >= 360.0)
              wheelVector.Angle -= 360.0;
        }
        else
        {
            // Determine if the wheel angle is greater than 180 degrees
            while (wheelVector.Angle <= -360.0)
                wheelVector.Angle += 360.0;

            // Handle special case (should not happen)
            if (wheelVector.Angle == -180.0)
               wheelVector.Angle = 180.0;
        }
    }
    else  // Targe angle less that 0 degrees (Compute angle in -180 to 0 degrees range)
    {
        // If the wheel angle is less that -180 degrees
        if (wheelVector.Angle < -180.0)
        {
           // Determine if the wheel angle is greater than 360 degrees
           while (wheelVector.Angle <= -360.0)
              wheelVector.Angle += 360.0;
        }
        else
        {
            // Determine if the wheel angle is greater than 360 degrees
            while (wheelVector.Angle >= 360.0)
                wheelVector.Angle -= 360.0;

            // Handle special case (should not happen)
            if (wheelVector.Angle == 180.0)
               wheelVector.Angle = -180.0;
        }
    }

    // Return the swerve angle in the proper hemisphere (-180 to 180 degrees)
    return wheelVector.Angle;
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
