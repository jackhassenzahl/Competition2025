#include "SwerveModule.h"

#pragma region SwerveModule Contructor
/// @brief Class constructor for the SwerveModule class.
/// @param driveMotorCanId The CAN ID for the swerve module drive motor.
/// @param angleMotorCanId The CAN ID for the swerve module angle motor.
/// @param angleEncoderCanId The CAN ID for the swerve module angle encoder.
SwerveModule::SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId) :
                           m_driveMotor(driveMotorCanId, CanConstants::CanBus),
                           m_angleMotor(angleMotorCanId, rev::spark::SparkLowLevel::MotorType::kBrushless),
                           m_angleAbsoluteEncoder(angleEncoderCanId, CanConstants::CanBus),
                           m_angleEncoder(m_angleMotor.GetEncoder())
{
    // Configure the drive and angle motors
    ConfigureDriveMotor();
    ConfigureAngleMotor();
}
#pragma endregion

#pragma region ConfigureDriveMotor
/// @brief Method to configure the drive motor.
void SwerveModule::ConfigureDriveMotor()
{
    // Create the drive motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration driveMotorConfiguration{};

    // Add the Motor Output section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = driveMotorConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    // Add the Current Limits section settings
    ctre::phoenix6::configs::CurrentLimitsConfigs &currentLimitsConfigs = driveMotorConfiguration.CurrentLimits;
    currentLimitsConfigs.StatorCurrentLimit       = ChassisConstants::SwerveDriveMaxAmperage;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    // Apply the configuration to the drive motor
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < ChassisConstants::MotorConfigurationAttempts; attempt++)
    {
        // Apply the configuration to the drive motor
        status = m_driveMotor.GetConfigurator().Apply(driveMotorConfiguration);

        // Check if the configuration was successful
        if (status.IsOK())
           break;
    }

    // Determine if the last configuration load was successful
    if (!status.IsOK())
        std::cout << "***** ERROR: Could not configure swerve motor. Error: " << status.GetName() << std::endl;
}
#pragma endregion

#pragma region ConfigureAngleMotor
/// @brief Method to configure the angle motor and encoder.
void SwerveModule::ConfigureAngleMotor()
{
    // Limit the PID Controller's input range between -pi and pi and set the input to be continuous
    m_turningPIDController.EnableContinuousInput(-units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});

    // Configure the angle motor
    rev::spark::SparkBaseConfig sparkBaseConfig{};
    sparkBaseConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    sparkBaseConfig.SecondaryCurrentLimit(ChassisConstants::SwerveAngleMaxAmperage);
    sparkBaseConfig.encoder.PositionConversionFactor(ChassisConstants::SwerveDegreesToMotorRevolutions).VelocityConversionFactor(1);
    sparkBaseConfig.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
                     .Pid(SwerveConstants::P, SwerveConstants::I, SwerveConstants::D);
    m_angleMotor.Configure(sparkBaseConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}
#pragma endregion

#pragma region SetWheelAngleToForward
/// @brief Method to set the swerve wheel encoder to the forward angle.
/// @param angle The absolute angle for the forward direction.
void SwerveModule::SetWheelAngleToForward(units::angle::degree_t forwardAngle)
{
    // Set the motor angle encoder position to the forward direction
    m_angleMotor.GetEncoder().SetPosition(GetAbsoluteAngle().value() - forwardAngle.value());
}
#pragma endregion

#pragma region SetState
/// @brief Set the swerve module angle and motor power.
/// @param vector The wheel vector (angle and drive).
void SwerveModule::SetState(frc::SwerveModuleState &referenceState)
{
    frc::Rotation2d encoderRotation{units::radian_t{m_angleEncoder.GetPosition()}};

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

    // Set the Drive motor power to zero
    m_driveMotor.SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
    m_angleMotor.SetVoltage(units::volt_t{turnOutput}  + turnFeedforward);
}

#pragma region GetAbsoluteAngle
/// @brief Method to read the absolute encode in Degrees.
/// @return The absolute angle value in degrees.
units::angle::degree_t SwerveModule::GetAbsoluteAngle()
{
    // The GetAbsolutePosition() method returns a value from -1 to 1
    double encoderValue = (double) m_angleAbsoluteEncoder.GetAbsolutePosition().GetValue();

    // To convert to degrees
    return encoderValue * 360_deg;
}
#pragma endregion

#pragma region GetDriveEncoderRate
/// @brief Method to retrieve the drive encoder rate (velocity in meters/s).
/// @return The wheel drive encoder rate.
units::meters_per_second_t SwerveModule::GetDriveEncoderRate()
{
    // Get the motor velocity
    double rotationsPerSecond = (double) m_driveMotor.GetVelocity().GetValue();

    // Return the wheel drive encoder rate
    return (units::meters_per_second_t) rotationsPerSecond;
}
#pragma endregion

#pragma region GetAngleEncoderDistance
/// @brief Method to retrieve the angle encoder distance (in radians).
units::radian_t SwerveModule::GetAngleEncoderDistance()
{
    // Return the angle encoder distance (position in radians)
    return (units::radian_t) m_angleEncoder.GetPosition();
}
#pragma endregion

#pragma region GetState
/// @brief  Method to retrieve the swerve module state.
/// @return The swerve module speed and angle state.
frc::SwerveModuleState SwerveModule::GetState()
{
    return {GetDriveEncoderRate(), units::radian_t{m_angleEncoder.GetPosition()}};
}
#pragma endregion

#pragma region GetPosition
/// @brief Method to retrieve the swerve module position.
frc::SwerveModulePosition SwerveModule::GetPosition()
{
    // Get the drive position
    double drivePosition = (double) m_driveMotor.GetPosition().GetValue();

    // Return the swerve module position
    return {units::meter_t{drivePosition}, units::radian_t{m_angleEncoder.GetPosition()}};
}
#pragma endregion
