#include "SwerveModule.h"

#pragma region SwerveModule Contructor
/// @brief Class constructor for the SwerveModule class.
/// @param driveMotorCanId The CAN ID for the swerve module drive motor.
/// @param angleMotorCanId The CAN ID for the swerve module angle motor.
/// @param angleEncoderCanId The CAN ID for the swerve module angle encoder.
SwerveModule::SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId)
{
    std::cout << "***** Swerve Module Constructor" << std::endl;
    std::cout << "   driveMotorCanId: " << driveMotorCanId << std::endl;
    std::cout << "   angleMotorCanId: " << angleMotorCanId << std::endl;

    // Configure the drive and angle motors
    ConfigureDriveMotor(driveMotorCanId);
    ConfigureAngleMotor(angleMotorCanId, angleEncoderCanId);
}
#pragma endregion

#pragma region ConfigureDriveMotor
/// @brief Method to configure the drive motor.
/// @param driveMotorCanId The drive motor CAN identification.
void SwerveModule::ConfigureDriveMotor(int driveMotorCanId)
{
    std::cout << "***** Configure Drive Motor: " << driveMotorCanId << std::endl;

    // Instantiate the drive motor
    m_driveMotor = new ctre::phoenix6::hardware::TalonFX{driveMotorCanId, CanConstants::CanBus};

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
        status = m_driveMotor->GetConfigurator().Apply(driveMotorConfiguration);

        // Check if the configuration was successful
        if (status.IsOK())
           break;
    }

    // Determine if the last configuration load was successful
    if (!status.IsOK())
        std::cout << "***** ERROR: Could not configure swerve motor. Error: " << status.GetName() << std::endl;

    std::cout << "***** GetDescription: " << m_driveMotor->GetDescription() << std::endl;
}
#pragma endregion

#pragma region ConfigureAngleMotor
/// @brief Method to configure the angle motor and encoder.
/// @param angleMotorCanId The angle motor CAN identification.
/// @param angleEncoderCanId The angle encoder CAN identification.
void SwerveModule::ConfigureAngleMotor(int angleMotorCanId, int angleEncoderCanId)
{
    std::cout << "***** Configure Angle Motor: " << angleMotorCanId << std::endl;

    // Instantiate the angle motor, encoder and absolute encode
    m_angleMotor           = new rev::spark::SparkMax{angleMotorCanId, rev::spark::SparkLowLevel::MotorType::kBrushless};
    m_angleEncoder         = new rev::spark::SparkRelativeEncoder(m_angleMotor->GetEncoder());
    //m_pidController        = new rev::spark::SparkClosedLoopController(m_angleMotor->GetClosedLoopController());
    m_angleAbsoluteEncoder = new ctre::phoenix6::hardware::CANcoder(angleEncoderCanId, CanConstants::CanBus);

    // Limit the PID Controller's input range between -pi and pi and set the input to be continuous
    m_turningPIDController.EnableContinuousInput(-units::radian_t{std::numbers::pi}, units::radian_t{std::numbers::pi});

    // Configure the angle motor
    rev::spark::SparkBaseConfig sparkBaseConfig{};
    sparkBaseConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    sparkBaseConfig.SecondaryCurrentLimit(ChassisConstants::SwerveAngleMaxAmperage);
    sparkBaseConfig.encoder.PositionConversionFactor(ChassisConstants::SwerveDegreesToMotorRevolutions).VelocityConversionFactor(1);
    sparkBaseConfig.closedLoop.SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
                     .Pid(ChassisConstants::SwerveP, ChassisConstants::SwerveI, ChassisConstants::SwerveD);
    m_angleMotor->Configure(sparkBaseConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    // Set the CAN coder absolute out range
    // Note: This is probably incorrect. Should be 0.5 (for -0.5 to 0.5) and will have to convert to degrees  TODO: Check angle values
    ctre::phoenix6::configs::CANcoderConfiguration canCoderConfiguration{};
    canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = -180_deg;
    m_angleAbsoluteEncoder->GetConfigurator().Apply(canCoderConfiguration);
}
#pragma endregion

#pragma region SetSwerveWheelAnglesToZero
/// @brief Method to set the swerve wheel to the specified angle.
/// @param angle The angle to set the wheel.
void SwerveModule::SetSwerveWheelAnglesToZero()
{
    // // Get the wheel absolute angle  TODO:
    // units::angle::degree_t absoluteAngle = GetAbsoluteAngle();

    // // Move the wheel to absolute encoder value
    // // Note: The reference angle is -1 to 1 so divide the degrees by 180
    // m_pidController->SetReference(absoluteAngle.value() / 180.0, rev::spark::SparkMax::ControlType::kPosition);

    // TODO: May need to wait for the wheel to reach the zero angle position

    // Set the angle encoder position to zero
    m_angleEncoder->SetPosition(0.0);
}
#pragma endregion

#pragma region SetState
/// @brief Set the swerve module angle and motor power.
/// @param vector The wheel vector (angle and drive).
void SwerveModule::SetState(frc::SwerveModuleState &referenceState)
{
    frc::Rotation2d encoderRotation{units::radian_t{m_angleEncoder->GetPosition()}};

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
    m_driveMotor->SetVoltage(units::volt_t{driveOutput} + driveFeedforward);
    m_angleMotor->SetVoltage(units::volt_t{turnOutput}  + turnFeedforward);
}

#pragma region GetDriveEncoderRate
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
#pragma endregion

#pragma region GetAngleEncoderDistance
/// @brief Method to retrieve the angle encoder distance (in radians).
units::radian_t SwerveModule::GetAngleEncoderDistance()
{
    // Return the angle encoder distance (position in radians)
    return (units::radian_t) m_angleEncoder->GetPosition();
}
#pragma endregion

#pragma region GetState
/// @brief  Method to retrieve the swerve module state.
/// @return The swerve module speed and angle state.
frc::SwerveModuleState SwerveModule::GetState()
{
    return {GetDriveEncoderRate(), units::radian_t{m_angleEncoder->GetPosition()}};
}
#pragma endregion

#pragma region GetPosition
/// @brief Method to retrieve the swerve module position.
frc::SwerveModulePosition SwerveModule::GetPosition()
{
    // Get the drive position
    double drivePosition = (double) m_driveMotor->GetPosition().GetValue();

    // Return the swerve module position
    return {units::meter_t{drivePosition}, units::radian_t{m_angleEncoder->GetPosition()}};
}
#pragma endregion
