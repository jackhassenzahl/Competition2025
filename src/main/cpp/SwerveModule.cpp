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

    // Initialize the angle and drive to zero
    m_wheelVector.Angle = 0.0;
    m_wheelVector.Drive = 0.0;

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
    m_pidController        = new rev::spark::SparkClosedLoopController(m_angleMotor->GetClosedLoopController());
    m_angleAbsoluteEncoder = new ctre::phoenix6::hardware::CANcoder(angleEncoderCanId, CanConstants::CanBus);

    // Configure the angle motor
    rev::spark::SparkBaseConfig sparkBaseConfig{};
    sparkBaseConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    sparkBaseConfig.SecondaryCurrentLimit(ChassisConstants::SwerveAngleMaxAmperage);
    sparkBaseConfig.encoder.PositionConversionFactor(1000).VelocityConversionFactor(1000);
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
    // Get the wheel absolute angle
    units::angle::degree_t absoluteAngle = GetAbsoluteAngle();

    // Move the wheel to absolute encoder value
    // Note: The reference angle is -1 to 1 so divide the degrees by 180
    m_pidController->SetReference(absoluteAngle.value() / 180.0, rev::spark::SparkMax::ControlType::kPosition);

    // TODO: May need to wait for the wheel to reach the zero angle position

    // Set the angle encoder position to zero
    m_angleEncoder->SetPosition(0.0);
}
#pragma endregion

#pragma region SetState
/// @brief Set the swerve module angle and motor power.
/// @param vector The wheel vector (angle and drive).
void SwerveModule::SetState(WheelVector vector)
{
    // Do not change the angle if the wheel is not driven
    if (vector.Drive > 0.01 || vector.Drive < -0.01)
    {
        // Optimize the serve module vector to minimize wheel rotation on change of diretion
        OptimizeWheelAngle(vector, &m_wheelVector);

        // Set the angle motor PID set angle
        m_pidController->SetReference(m_wheelVector.Angle * ChassisConstants::SwerveWheelCountsPerRevoplution, rev::spark::SparkMax::ControlType::kPosition);
    }
    else
    {
        // Ensure the drive motor is disabled
        m_wheelVector.Drive = 0.0;
    }

    // Set the Drive motor voltage
    m_driveMotor->SetControl(m_voltageOut.WithOutput(m_wheelVector.Drive * 12_V));
}
#pragma endregion

#pragma region OptimizeWheelAngle
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
#pragma endregion

#pragma region ConvertAngleToTargetRange
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
#pragma endregion

#pragma region GetAbsoluteAngle
/// @brief Method to read the absolute encode in Degrees.
/// @return The absolute angle value in degrees.
units::angle::degree_t SwerveModule::GetAbsoluteAngle()
{
    // The GetAbsolutePosition() method returns a value from -1 to 1
    double encoderValue = (double) m_angleAbsoluteEncoder->GetAbsolutePosition().GetValue();

    // To convert to degrees (-180 to 180) -> multiply to 180 degrees
    return encoderValue * 180_deg;
}
#pragma endregion

#pragma region GetWheelVector
/// <summary>
/// Method to get the swerve module wheel vector.
/// </summary>
/// <param name="wheelVector">Variable to return the swerve module wheel vector.</param>
WheelVector* SwerveModule::GetWheelVector()
{
    // Return the wheel vector
    return &m_wheelVector;
}
#pragma endregion
