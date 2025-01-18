#include <cmath>

#include <rev/SparkMax.h>
#include <ctre/phoenix6/configs/Configs.hpp>

#include <frc/geometry/Rotation2d.h>

#include "SwerveModule.h"

/// @brief Class constructor for the SwerveModule class.
/// @param driveMotorCanId The CAN ID for the swerve module drive motor.
/// @param angleMotorCanId The CAN ID for the swerve module angle motor.
/// @param angleEncoderCanId The CAN ID for the swerve module angle encoder.
SwerveModule::SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId)
{
#if defined(ROBOT)
    // Configure the drive and angle motors
    ConfigureDriveMotor(driveMotorCanId);
    ConfigureAngleMotor(angleMotorCanId, angleEncoderCanId);
#endif
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

    // Set the current limit
    ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitsConfigs{};
    currentLimitsConfigs.StatorCurrentLimit       = ChassisConstants::kSwerveDriveMaxAmperage;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    m_driveMotor->GetConfigurator().Apply(currentLimitsConfigs);
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

    // Limit the PID Controller's input range between -pi and pi and set the input to be continuous.
    m_turningPIDController.EnableContinuousInput(-units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});

    // Set the absolute out range
    // Note: This is probably incorrect. Should be 0.5 (for -0.5 to 0.5) and will have to convert to degrees
    ctre::phoenix6::configs::CANcoderConfiguration toApply{};
    toApply.MagnetSensor.AbsoluteSensorDiscontinuityPoint = -180_deg;
    m_angleAbsoluteEncoder->GetConfigurator().Apply(toApply);

    // Configure the angle motor
    rev::spark::SparkBaseConfig config{};

    config.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    config.SecondaryCurrentLimit(ChassisConstants::kSwerveAngleMaxAmperage);
    config.encoder.PositionConversionFactor(1000).VelocityConversionFactor(1000);

    m_angleMotor->Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

/// @brief Set the swerve module angle and motor power.
/// @param vector The wheel vector (angle and drive).
void SwerveModule::SetState(frc::SwerveModuleState &referenceState)
{
#if defined(ROBOT)    
    frc::Rotation2d encoderRotation{units::radian_t{m_angleEncoder->GetPosition()}};
#else
    frc::Rotation2d encoderRotation{units::radian_t{0}};
#endif

    // Optimize the reference state to avoid spinning further than 90 degrees
    referenceState.Optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired direction of travel that can occur when
    // modules change directions. This results in smoother driving.
    referenceState.CosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller.
    const auto driveOutput      = m_drivePIDController.Calculate(GetDriveEncoderRate().value(), referenceState.speed.value());
    const auto driveFeedforward = m_driveFeedforward.Calculate(referenceState.speed);

    // Calculate the turning motor output from the turning PID controller.
    const auto turnOutput      = m_turningPIDController.Calculate(GetAngleEncoderDistance(), referenceState.angle.Radians());
    const auto turnFeedforward = m_turnFeedforward.Calculate(m_turningPIDController.GetSetpoint().velocity);

#if defined(ROBOT)
    // Set the motor outputs.
    m_driveMotor->SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
    m_angleMotor->SetVoltage(units::volt_t{turnOutput}  + turnFeedforward);
#endif
}

/// @brief Method to retrieve the drive encoder rate (velocity in meters/s).
/// @return The wheel drive encoder rate.
units::meters_per_second_t SwerveModule::GetDriveEncoderRate()
{
#if defined(ROBOT)
    // Get the motor velocity
    double rotationsPerSecond = (double) m_driveMotor->GetVelocity().GetValue();

    // Return the wheel drive encoder rate
    return (units::meters_per_second_t) rotationsPerSecond;
#else
    return (units::meters_per_second_t) 0;
#endif
}

/// @brief Method to retrieve the angle encoder distance (in radians).
units::radian_t SwerveModule::GetAngleEncoderDistance()
{
#if defined(ROBOT)
    // Return the angle encoder distance (position in radians)
    return (units::radian_t) m_angleEncoder->GetPosition();
#else
    return (units::radian_t) 0;
#endif
}

/// @brief  Method to retrieve the swerve module state.
/// @return The swerve module speed and angle state.
frc::SwerveModuleState SwerveModule::GetState()
{
#if defined(ROBOT)
    return {GetDriveEncoderRate(), units::radian_t{m_angleEncoder->GetPosition()}};
#else
    return {units::meters_per_second_t{0}, units::radian_t{0}};
#endif
}

/// @brief Method to retrieve the swerve module position.
frc::SwerveModulePosition SwerveModule::GetPosition()
{
#if defined(ROBOT)
    // Get the drive position
    double drivePosition = (double) m_driveMotor->GetPosition().GetValue();
    
    // Return the swerve module position
    return {units::meter_t{drivePosition}, units::radian_t{m_angleEncoder->GetPosition()}};
#else
    return {units::meter_t{0}, units::radian_t{0}};
#endif
}
