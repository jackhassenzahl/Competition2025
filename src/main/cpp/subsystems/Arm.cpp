#include "subsystems/Arm.h"

#pragma region Arm (constructor)
/// @brief Class to support the Arm subsystem.
Arm::Arm()
{
    // Configure the Arm motor
    ConfigureArmMotor(CanConstants::ArmMotorCanId);
}
#pragma endregion

#pragma region ConfigureArmMotor
/// @brief Method to configure the Arm motor using MotionMagic.
/// @param motorCanId The CAN identifier for the Arm motor.
void Arm::ConfigureArmMotor(int motorCanId)
{
    // Instantiate the Arm motor
    m_armMotor = new ctre::phoenix6::hardware::TalonFX{motorCanId, CanConstants::CanBus};

    // Create the Arm motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration armMotorConfiguration{};

    // Add the Motor Output section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = armMotorConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    ctre::phoenix6::configs::Slot0Configs &slot0Configs = armMotorConfiguration.Slot0;
    slot0Configs.kS = ArmConstants::S;
    slot0Configs.kV = ArmConstants::V;
    slot0Configs.kA = ArmConstants::A;
    slot0Configs.kP = ArmConstants::P;
    slot0Configs.kI = ArmConstants::I;
    slot0Configs.kD = ArmConstants::D;

    // // Configure gear ratio
    // ctre::phoenix6::configs::FeedbackConfigs &feedbackConfigs = ArmMotorConfiguration.Feedback;
    // feedbackConfigs.SensorToMechanismRatio = ArmConstants::SensorToMechanismRatio;

    // Configure Motion Magic
    ctre::phoenix6::configs::MotionMagicConfigs &motionMagicConfigs = armMotorConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants::MotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration   = ArmConstants::MotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk           = ArmConstants::MotionMagicJerk;

    // Apply the configuration to the drive motor
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < CanConstants::MotorConfigurationAttempts; attempt++)
    {
        // Apply the configuration to the drive motor
        status = m_armMotor->GetConfigurator().Apply(armMotorConfiguration);

        // Check if the configuration was successful
        if (status.IsOK())
           break;
    }

    // Determine if the last configuration load was successful
    if (!status.IsOK())
        std::cout << "***** ERROR: Could not configure arm motor. Error: " << status.GetName() << std::endl;
}
#pragma endregion

#pragma region SetAngle
/// @brief Method to set the arm angle.
/// @param position The setpoint for the arm angle. Takes -180 -> 180
void Arm::SetAngle(units::angle::degree_t angle)
{
    // Compute the number of turns based on the specficied angle
    units::angle::turn_t newPosition = (units::angle::turn_t) (angle.value() / 360.0);

    // Set the arm set position
    m_armMotor->SetControl(m_motionMagicVoltage.WithPosition(newPosition).WithSlot(0));
}
#pragma endregion
