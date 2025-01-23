#include "subsystems/Elevator.h"

#pragma region Elevator (constructor)
/// @brief Class to support the elevator subsystem.
Elevator::Elevator()
{
    // Configure the elevator motor
    ConfigureElevatorMotor(CanConstants::ElevatorMotorCanId);
}
#pragma endregion

#pragma region ConfigureElevatorMotor
/// @brief Method to configure the elevator motor using MotionMagic.
/// @param driveMotorCanId The CAN identifier for the elevator motor.
void Elevator::ConfigureElevatorMotor(int driveMotorCanId)
{
    std::cout << "***** Configure Drive Motor: " << driveMotorCanId << std::endl;

    // Instantiate the drive motor
    m_elevatorMotor = new ctre::phoenix6::hardware::TalonFX{driveMotorCanId, CanConstants::CanBus};

    // Create the drive motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration elevatorMotorConfiguration{};

    // Add the Motor Output section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = elevatorMotorConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    ctre::phoenix6::configs::Slot0Configs &slot0Configs = elevatorMotorConfiguration.Slot0;
    slot0Configs.kS = 0.25;  // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12;  // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01;  // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP =   60;  // A position error of 0.2 rotations results in 12 V output
    slot0Configs.kI =    0;  // No output for integrated error
    slot0Configs.kD =  0.5;  // A velocity error of 1 rps results in 0.5 V output

    // Configure gear ratio
    ctre::phoenix6::configs::FeedbackConfigs &feedbackConfigs = elevatorMotorConfiguration.Feedback;
    feedbackConfigs.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation

    // Configure Motion Magic
    ctre::phoenix6::configs::MotionMagicConfigs &motionMagicConfigs = elevatorMotorConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 5_tps;            // 5 (mechanism) rotations per second cruise
    motionMagicConfigs.MotionMagicAcceleration   = 10_tr_per_s_sq;   // Take approximately 0.5 seconds to reach max vel
    motionMagicConfigs.MotionMagicJerk           = 100_tr_per_s_cu;  // Take approximately 0.1 seconds to reach max accel

    // Apply the configuration to the drive motor
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < ChassisConstants::MotorConfigurationAttempts; attempt++)
    {
        // Apply the configuration to the drive motor
        status = m_elevatorMotor->GetConfigurator().Apply(elevatorMotorConfiguration);

        // Check if the configuration was successful
        if (status.IsOK())
           break;
    }

    // Determine if the last configuration load was successful
    if (!status.IsOK())
        std::cout << "***** ERROR: Could not configure elevator motor. Error: " << status.GetName() << std::endl;

    std::cout << "***** GetDescription: " << m_elevatorMotor->GetDescription() << std::endl;
}
#pragma endregion

#pragma region SetHeight
/// @brief Method to set the elevator height.
/// @param position The setpoint for the elevator height.
void Elevator::SetHeight(units::length::meter_t position)
{
    // Set the elevator set position
    //m_elevatorMotor->SetControl(m_motionMagicVoltage.WithPosition(position).WithSlot(0)); TODO: Fix
}
#pragma endregion
