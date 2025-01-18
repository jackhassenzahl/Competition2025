#include <cmath>

#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <rev/SparkMax.h>

#include "Constants.h"
#include "SwerveModule.h"

/// @brief Class constructor for the SwerveModule class.
/// @param driveMotorCanId The CAN ID for the swerve module drive motor.
/// @param angleMotorCanId The CAN ID for the swerve module angle motor.
/// @param angleEncoderCanId The CAN ID for the swerve module angle encoder.
SwerveModule::SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId)
{
    // Initialize the angle and drive to zero
    m_wheelVector.Angle = 0.0;
    m_wheelVector.Drive = 0.0;
 
    // Configure the drive and angle motors
    ConfigureDriveMotor(driveMotorCanId);
    ConfigureAngleMotor(angleMotorCanId, angleEncoderCanId);
}

/// @brief Method to configure the drive motor.
/// @param driveMotorCanId The drive motor CAN identification.
void SwerveModule::ConfigureDriveMotor(int driveMotorCanId)
{
    // Instantiate the drive motor
    ctre::phoenix6::hardware::TalonFX driveMotor = ctre::phoenix6::hardware::TalonFX{driveMotorCanId, CanConstants::kCanBus};
    m_driveMotor = &driveMotor;

    // Configure the drive motors 
    ctre::phoenix6::configs::TalonFXConfiguration swerve_motor_configuration{};

    ctre::phoenix6::configs::Slot0Configs slot0Configs = swerve_motor_configuration.Slot0;
    slot0Configs.kP = ChassisConstants::kSwerveP;  // An error of 0.5 rotations results in 12 V output
    slot0Configs.kI = ChassisConstants::kSwerveI;  // no output for integrated error
    slot0Configs.kD = ChassisConstants::kSwerveD;  // A velocity of 1 rps results in 0.1 V output

    //m_driveMotor->GetConfigurator().Apply(swerve_motor_configuration);
    
    // Set the current limit
    ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitsConfigs{};
    currentLimitsConfigs.StatorCurrentLimit       = ChassisConstants::kSwerveDriveMaxAmperage;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    //m_driveMotor->GetConfigurator().Apply(currentLimitsConfigs);
}

/// @brief Method to configure the angle motor and encoder.
/// @param angleMotorCanId The angle motor CAN identification.
/// @param angleEncoderCanId The angle encoder CAN identification.
void SwerveModule::ConfigureAngleMotor(int angleMotorCanId, int angleEncoderCanId)
{
    // Instantiate the angle motor
    rev::spark::SparkMax angleMotor = rev::spark::SparkMax{angleMotorCanId, rev::spark::SparkLowLevel::MotorType::kBrushless};
    m_angleMotor = &angleMotor;

    // Create the angle encoder based initialized with the present angle motor encoder value
    m_angleEncoder = new rev::spark::SparkRelativeEncoder(m_angleMotor->GetEncoder());

    ctre::phoenix6::hardware::CANcoder angleAbsoluteEncoder = ctre::phoenix6::hardware::CANcoder(angleEncoderCanId, CanConstants::kCanBus);
    m_angleAbsoluteEncoder = &angleAbsoluteEncoder;

    // Set the absolute out range
    // Note: This is probably incorrect. Should be 0.5 (for -0.5 to 0.5) and will have to convert to degrees
    ctre::phoenix6::configs::CANcoderConfiguration toApply{};
    toApply.MagnetSensor.AbsoluteSensorDiscontinuityPoint = -180_deg;
    //m_angleAbsoluteEncoder->GetConfigurator().Apply(toApply);

    // Configure the angle motor
    rev::spark::SparkBaseConfig config{};

    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    config.SecondaryCurrentLimit(ChassisConstants::kSwerveAngleMaxAmperage);
    config.encoder.PositionConversionFactor(1000).VelocityConversionFactor(1000);
    config.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
                     .Pid(ChassisConstants::kSwerveP, ChassisConstants::kSwerveI, ChassisConstants::kSwerveD);
 
    m_angleMotor->Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

/// @brief Set the swerve module angle and motor power.
/// @param vector The wheel vector (angle and drive).
void SwerveModule::SetState(WheelVector vector)
{
    // Do not change the angle if the wheel is not driven
    if (vector.Drive > 0.0)
    {
        // Optimize the serve module vector to minimize wheel rotation on change of diretion
       OptimizeWheelAngle(vector, &m_wheelVector);

#if defined(ROBOT)
        // Set the Drive motor power
        m_driveMotor->Set(m_wheelVector.Drive);

        // Set the angle motor PID set angle
       m_pidController->SetReference(m_wheelVector.Angle * ChassisConstants::kSwerveWheelCountsPerRevoplution, rev::spark::SparkMax::ControlType::kPosition);
#endif    
    }
    else
    {
        // Ensure the drive motor is disabled
        m_wheelVector.Drive = 0.0;

#if defined(ROBOT)
        // Set the Drive motor power to zero
        m_driveMotor->Set(m_wheelVector.Drive);
#endif  
    }
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
WheelVector* SwerveModule::GetWheelVector()
{
    // Return the wheel vector
    return &m_wheelVector;
}
