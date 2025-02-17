#include "subsystems/Climb.h"

#pragma region Climb (constructor)
/// @brief Class to support the Climb subsystem.
Climb::Climb()
{
    // Configure the climb motor
    ConfigureClimbMotor(CanConstants::ClimbMotorCanId);
}
#pragma endregion

#pragma region ConfigureClimbMotor
/// @brief Method to configure the climb motor using MotionMagic.
/// @param motorCanId The CAN identifier for the climb motor.
void Climb::ConfigureClimbMotor(int motorCanId)
{
    // Instantiate the climb motor
    m_climbMotor = new ctre::phoenix6::hardware::TalonFX{motorCanId, CanConstants::CanBus};

    // Create the climb motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration climbMotorConfiguration{};

    // Add the Motor Output section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = climbMotorConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    ctre::phoenix6::configs::Slot0Configs &slot0Configs = climbMotorConfiguration.Slot0;
    slot0Configs.kS = ClimbConstants::S;
    slot0Configs.kV = ClimbConstants::V;
    slot0Configs.kA = ClimbConstants::A;
    slot0Configs.kP = ClimbConstants::P;
    slot0Configs.kI = ClimbConstants::I;
    slot0Configs.kD = ClimbConstants::D;

    // // Configure gear ratio
    // ctre::phoenix6::configs::FeedbackConfigs &feedbackConfigs = ClimbMotorConfiguration.Feedback;
    // feedbackConfigs.SensorToMechanismRatio = ClimbConstants::SensorToMechanismRatio;

    // Configure Motion Magic
    ctre::phoenix6::configs::MotionMagicConfigs &motionMagicConfigs = climbMotorConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ClimbConstants::MotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration   = ClimbConstants::MotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk           = ClimbConstants::MotionMagicJerk;

    // Apply the configuration to the drive motor
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < CanConstants::MotorConfigurationAttempts; attempt++)
    {
        // Apply the configuration to the drive motor
        status = m_climbMotor->GetConfigurator().Apply(climbMotorConfiguration);

        // Check if the configuration was successful
        if (status.IsOK())
           break;
    }

    // Determine if the last configuration load was successful
    if (!status.IsOK())
        std::cout << "***** ERROR: Could not configure climb motor. Error: " << status.GetName() << std::endl;

    // Set the climb motor control to the default
    SetAngle(0_deg);
}
#pragma endregion

#pragma region SetAngle
/// @brief Method to set the climb angle.
/// @param position The setpoint for the climb angle.
void Climb::SetAngle(units::angle::degree_t angle)
{
    // // Making sure that the climb doesn't try to go through the robot
    // if (angle < ClimbConstants::MinimumPosition)  // TODO: Need to calibrate angle to motor rotations
    //    angle = ClimbConstants::MinimumPosition;

    // if (angle > ClimbConstants::MaximumPosition)
    //     angle = ClimbConstants::MaximumPosition;

    // Compute the number of turns based on the specficied angle
    units::angle::turn_t newPosition = (units::angle::turn_t) (angle.value() * ClimbConstants::AngleToTurnsConversionFactor.value());

    // Set the climb set position
    m_climbMotor->SetControl(m_motionMagicVoltage.WithPosition(newPosition).WithSlot(0));
}
#pragma endregion

#pragma region GetAngle
/// @brief Method to get the climb angle.
/// @return The climb angle.
units::angle::degree_t Climb::GetAngle()
{
    // Get the current climb motor angle
    auto currentAngle = m_climbMotor->GetPosition().GetValueAsDouble();

    // Return the climb angle
    return (units::angle::degree_t) (currentAngle / ClimbConstants::AngleToTurnsConversionFactor.value());
}
#pragma endregion
