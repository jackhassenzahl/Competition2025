#include "SwerveModule.h"

#pragma region SwerveModule Contructor
/// @brief Class constructor for the SwerveModule class.
/// @param driveMotorCanId The CAN ID for the swerve module drive motor.
/// @param angleMotorCanId The CAN ID for the swerve module angle motor.
/// @param angleEncoderCanId The CAN ID for the swerve module angle encoder.
/// @param chassisAngularOffset The offset of the swerve module to the chassis.
SwerveModule::SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId, double chassisAngularOffset) :
                           m_driveMotor(driveMotorCanId, CanConstants::CanBus),
                           m_angleMotor(angleMotorCanId, rev::spark::SparkMax::MotorType::kBrushless),

                           m_chassisAngularOffset(chassisAngularOffset),
                           m_angleAbsoluteEncoder(angleEncoderCanId, CanConstants::CanBus)
{
    // Configure the drive and angle motors
    ConfigureDriveMotor();
    ConfigureAngleMotor();

    m_desiredState.angle = frc::Rotation2d(units::radian_t{m_turnAbsoluteEncoder.GetPosition()});
    m_driveMotor.SetPosition(0_tr);
}
#pragma endregion

#pragma region ConfigureDriveMotor
/// @brief Method to configure the drive motor.
void SwerveModule::ConfigureDriveMotor()
{
    // Use module constants to calculate conversion factors and feed forward gain.
    double drivingFactor = ModuleConstants::WheelDiameter.value() *
                           std::numbers::pi / ModuleConstants::DrivingMotorReduction;

    double drivingVelocityFeedForward = 1 / ModuleConstants::DriveWheelFreeSpeedRps;

    // drivingConfig
    //     .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
    //     .SmartCurrentLimit(50);
    // drivingConfig.encoder
    //     .PositionConversionFactor(drivingFactor)          // meters
    //     .VelocityConversionFactor(drivingFactor / 60.0);  // meters per second
    // drivingConfig.closedLoop
    //     .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
    //     // These are example gains you may need to them for your own robot!
    //     .Pid(0.04, 0, 0)
    //     .VelocityFF(drivingVelocityFeedForward)
    //     .OutputRange(-1, 1);

    // Create the drive motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfiguration{};

    // Add the "Motor Output" section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = talonFXConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    // Add the "Current Limits" section settings
    ctre::phoenix6::configs::CurrentLimitsConfigs &currentLimitsConfigs = talonFXConfiguration.CurrentLimits;
    currentLimitsConfigs.StatorCurrentLimit       = ChassisConstants::SwerveDriveMaxAmperage;
    currentLimitsConfigs.StatorCurrentLimitEnable = true;

    // Add the "Slot0" section settings
    ctre::phoenix6::configs::Slot0Configs &slot0Configs = talonFXConfiguration.Slot0;
    slot0Configs.kP = SwerveConstants::DriveP;
    slot0Configs.kI = SwerveConstants::DriveI;
    slot0Configs.kD = SwerveConstants::DriveD;
    slot0Configs.kV = drivingVelocityFeedForward;

    // Add the "Feedback" section settings
    ctre::phoenix6::configs::FeedbackConfigs &feedbackConfigs = talonFXConfiguration.Feedback;
    feedbackConfigs.SensorToMechanismRatio = drivingFactor;

    // Apply the configuration to the drive motor
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < ChassisConstants::MotorConfigurationAttempts; attempt++)
    {
        // Apply the configuration to the drive motor
        status = m_driveMotor.GetConfigurator().Apply(talonFXConfiguration);

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
    static rev::spark::SparkMaxConfig sparkMaxConfig{};

    // Use module constants to calculate conversion factor
    double turningFactor = 2 * std::numbers::pi;

    sparkMaxConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(ChassisConstants::SwerveAngleMaxAmperage);
    sparkMaxConfig.absoluteEncoder
        // Invert the turning encoder, since the output shaft rotates in the
        // opposite direction of the steering motor in the MAXSwerve Module.
        .Inverted(true)
        .PositionConversionFactor(turningFactor)          // radians
        .VelocityConversionFactor(turningFactor / 60.0);  // radians per second
    sparkMaxConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder)
        .Pid(SwerveConstants::AngleP, SwerveConstants::AngleI, SwerveConstants::AngleD)
        .OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, turningFactor);

    // Write the configuration to the motor controller
    m_angleMotor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}
#pragma endregion

#pragma region GetState
/// @brief  Method to retrieve the swerve module state.
/// @return The swerve module speed and angle state.
frc::SwerveModuleState SwerveModule::GetState()
{
    // Determine the module wheel velocity
    double velocity = (double) m_driveMotor.GetVelocity().GetValue() / ChassisConstants::DriveMotorVelocityConversion;  // TODO: Determine DriveMotorVelocityConversion

    // Return the swerve module state
    return {units::meters_per_second_t {velocity},
            units::radian_t{m_turnAbsoluteEncoder.GetPosition() - m_chassisAngularOffset}};
}
#pragma endregion

#pragma region GetPosition
/// @brief Method to retrieve the swerve module position.
frc::SwerveModulePosition SwerveModule::GetPosition()
{
    // Determine the module wheel position
    double position = (double) m_driveMotor.GetPosition().GetValue();

    // Return the swerve module position
    return {units::meter_t{position},
            units::radian_t{m_turnAbsoluteEncoder.GetPosition() - m_chassisAngularOffset}};
}
#pragma endregion

#pragma region SetDesiredState
/// @brief Method to set the swerve module state to the desired state.
/// @param desiredState The desired swerve module velocity and angle.
/// @param description String to show the module state on the SmartDashboard.
void SwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState, std::string description)
{
    // Apply chassis angular offset to the desired state.
    frc::SwerveModuleState correctedDesiredState{};
    correctedDesiredState.speed = desiredState.speed;
    correctedDesiredState.angle = desiredState.angle + frc::Rotation2d(units::radian_t{m_chassisAngularOffset});

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.Optimize(frc::Rotation2d(units::radian_t{m_turnAbsoluteEncoder.GetPosition()}));

    frc::SmartDashboard::PutString("Debug", "SetDesiredState: " + description);

    frc::SmartDashboard::PutNumber(description + "Drive", (double) correctedDesiredState.speed );
    frc::SmartDashboard::PutNumber(description + "Angle", (double) correctedDesiredState.angle.Degrees().value());

    //m_driveClosedLoopController.SetReference((double)correctedDesiredState.speed, rev::spark::SparkMax::ControlType::kVelocity);  TODO: Set the drive motor state
    m_turnClosedLoopController.SetReference(correctedDesiredState.angle.Radians().value(), rev::spark::SparkMax::ControlType::kPosition);

    // Remeber the desired state
    m_desiredState = desiredState;
}
#pragma endregion

#pragma region ResetEncoders
// Reset the drive encoder position.
void SwerveModule::ResetEncoders()
{
    m_driveMotor.SetPosition(0_tr);
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
