#include "subsystems/Gripper.h"

#pragma region Gripper
/// @brief The Constructor for the Gripper class.
Gripper::Gripper() : m_armMotor(CanConstants::ArmMotorCanId, rev::spark::SparkMax::MotorType::kBrushless),
                     m_armEncoder(m_wristMotor.GetEncoder()),
                     m_armTurnClosedLoopController(m_wristMotor.GetClosedLoopController()),

                     m_wristMotor(CanConstants::WristMotorCanId, rev::spark::SparkMax::MotorType::kBrushless),
                     m_wristEncoder(m_wristMotor.GetEncoder()),
                     m_wristTurnClosedLoopController(m_wristMotor.GetClosedLoopController()),

                     m_gripperMotorRight(CanConstants::GripperMotorCanIdRight, rev::spark::SparkMax::MotorType::kBrushless),
                     m_gripperMotorLeft(CanConstants::GripperMotorCanIdLeft,   rev::spark::SparkMax::MotorType::kBrushless)
{
    // Configure the elevator motor
    ConfigureElevatorMotor(CanConstants::ElevatorMotorCanId);

    // Configure the Arm motor
    ConfigureArmMotorTalon(CanConstants::ArmMotorCanId); // TODO: Use for TalonFx
    // ConfigureArmMotor();

    // Configure the gripper and wrist motors
    ConfigureWristMotor();
    ConfigureGripperMotorRight();
    ConfigureGripperMotorLeft();
}
#pragma endregion

#pragma region ConfigureElevatorMotor
/// @brief Method to configure the elevator motor using MotionMagic.
/// @param motorCanId The CAN identifier for the elevator motor.
void Gripper::ConfigureElevatorMotor(int motorCanId)
{
    // Instantiate the elevator motor
    m_elevatorMotor = new ctre::phoenix6::hardware::TalonFX{motorCanId, CanConstants::CanBus};

    // Create the elevator motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration elevatorMotorConfiguration{};

    // Add the Motor Output section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = elevatorMotorConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    motorOutputConfigs.Inverted    = true;

    ctre::phoenix6::configs::Slot0Configs &slot0Configs = elevatorMotorConfiguration.Slot0;
    slot0Configs.kS = ElevatorConstants::S;
    slot0Configs.kV = ElevatorConstants::V;
    slot0Configs.kA = ElevatorConstants::A;
    slot0Configs.kP = ElevatorConstants::P;
    slot0Configs.kI = ElevatorConstants::I;
    slot0Configs.kD = ElevatorConstants::D;

    // // Configure gear ratio
    // ctre::phoenix6::configs::FeedbackConfigs &feedbackConfigs = elevatorMotorConfiguration.Feedback;
    // feedbackConfigs.SensorToMechanismRatio = ElevatorConstants::SensorToMechanismRatio;

    // Configure Motion Magic
    ctre::phoenix6::configs::MotionMagicConfigs &motionMagicConfigs = elevatorMotorConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants::MotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration   = ElevatorConstants::MotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk           = ElevatorConstants::MotionMagicJerk;

    // Apply the configuration to the drive motor
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < CanConstants::MotorConfigurationAttempts; attempt++)
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

    // Set the elevator motor control to the default
    SetElevatorHeight(0_m);
}
#pragma endregion

#pragma region ConfigureArmMotorTalon
/// @brief Method to configure the Arm motor using MotionMagic.
/// @param motorCanId The CAN identifier for the Arm motor.
void Gripper::ConfigureArmMotorTalon(int motorCanId)
{
    // Instantiate the Arm motor
    m_armMotorTalon = new ctre::phoenix6::hardware::TalonFX{motorCanId, CanConstants::CanBus};

    // Create the Arm motor configuration
    ctre::phoenix6::configs::TalonFXConfiguration armMotorConfiguration{};

    // Add the Motor Output section settings
    ctre::phoenix6::configs::MotorOutputConfigs &motorOutputConfigs = armMotorConfiguration.MotorOutput;
    motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    ctre::phoenix6::configs::Slot0Configs &slot0Configs = armMotorConfiguration.Slot0;
    slot0Configs.kS = ArmConstantsTalon::S;
    slot0Configs.kV = ArmConstantsTalon::V;
    slot0Configs.kA = ArmConstantsTalon::A;
    slot0Configs.kP = ArmConstantsTalon::P;
    slot0Configs.kI = ArmConstantsTalon::I;
    slot0Configs.kD = ArmConstantsTalon::D;

    // // Configure gear ratio
    // ctre::phoenix6::configs::FeedbackConfigs &feedbackConfigs = ArmMotorConfiguration.Feedback;
    // feedbackConfigs.SensorToMechanismRatio = ArmConstants::SensorToMechanismRatio;

    // Configure Motion Magic
    ctre::phoenix6::configs::MotionMagicConfigs &motionMagicConfigs = armMotorConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstantsTalon::MotionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration   = ArmConstantsTalon::MotionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk           = ArmConstantsTalon::MotionMagicJerk;

    // Apply the configuration to the drive motor
    ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int attempt = 0; attempt < CanConstants::MotorConfigurationAttempts; attempt++)
    {
        // Apply the configuration to the drive motor
        status = m_armMotorTalon->GetConfigurator().Apply(armMotorConfiguration);

        // Check if the configuration was successful
        if (status.IsOK())
           break;
    }

    // Determine if the last configuration load was successful
    if (!status.IsOK())
        std::cout << "***** ERROR: Could not configure arm motor. Error: " << status.GetName() << std::endl;
}
#pragma endregion

#pragma region ConfigureArmMotor
/// @brief Method to configure the Arm motor using MotionMagic.
void Gripper::ConfigureArmMotor()
{
    // Configure the arm motor
    static rev::spark::SparkMaxConfig sparkMaxConfig{};

    sparkMaxConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(ArmConstants::MaxAmperage);
    sparkMaxConfig.encoder
        //.Inverted(true)Arm
        .PositionConversionFactor(ArmConstants::RadiansToMotorRevolutions)
        .VelocityConversionFactor(ArmConstants::RadiansToMotorRevolutions / 60.0);
    sparkMaxConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(ArmConstants::P, ArmConstants::I, ArmConstants::D)
        //.OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, 2 * std::numbers::pi);

    // Write the configuration to the motor controller
    m_armMotor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}
#pragma endregion

#pragma region ConfigureWristMotor
/// @brief Method to configure the Wrist motor using MotionMagic.
void Gripper::ConfigureWristMotor()
{
    // Configure the wrist motor
    static rev::spark::SparkMaxConfig sparkMaxConfig{};

    sparkMaxConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(WristConstants::MaxAmperage);
    sparkMaxConfig.encoder
        //.Inverted(true)
        .PositionConversionFactor(WristConstants::RadiansToMotorRevolutions)
        .VelocityConversionFactor(WristConstants::RadiansToMotorRevolutions / 60.0);
    sparkMaxConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(WristConstants::P, WristConstants::I, WristConstants::D)
        //.OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, 2 * std::numbers::pi);

    // Write the configuration to the motor controller
    m_wristMotor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}
#pragma endregion

#pragma region ConfigureGripperMotorRight
/// @brief Method to configure the Gripper motor using MotionMagic.
void Gripper::ConfigureGripperMotorRight()
{
    // Configure the angle motor
    static rev::spark::SparkMaxConfig sparkMaxConfig{};

    sparkMaxConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(GripperConstants::GripperMaxAmperage);
    sparkMaxConfig.encoder
        .PositionConversionFactor(2.0 * std::numbers::pi)
        .VelocityConversionFactor(1);
    sparkMaxConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(GripperConstants::GripperP, GripperConstants::GripperI, GripperConstants::GripperD)
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, 2 * std::numbers::pi);

    // Write the configuration to the motor controller
    m_gripperMotorRight.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}
#pragma endregion

#pragma region ConfigureGripperMotorLeft
/// @brief Method to configure the Gripper motor using MotionMagic.
void Gripper::ConfigureGripperMotorLeft()
{
    // Configure the angle motor
    static rev::spark::SparkMaxConfig sparkMaxConfig{};

    sparkMaxConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(GripperConstants::GripperMaxAmperage);
    sparkMaxConfig.encoder
        .PositionConversionFactor(2.0 * std::numbers::pi)
        .VelocityConversionFactor(1);
    sparkMaxConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(GripperConstants::GripperP, GripperConstants::GripperI, GripperConstants::GripperD)
        .PositionWrappingEnabled(true)
        .PositionWrappingInputRange(0, 2 * std::numbers::pi);

    // Write the configuration to the motor controller
    m_gripperMotorLeft.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}
#pragma endregion

#pragma region SetPose
/// @brief Method to set the pose of the gripper.
/// @param pose The pose to set the gripper.
void Gripper::SetPose(GripperPoseEnum pose)
{
    auto elevatorHeight  = 0_m;
    auto armAngle        = 0_deg;
    auto wristAngle      = 0_deg;
    auto gripperVoltage  = 0.0_V;

    // Determine the pose
    switch (pose)
    {
        case GripperPoseEnum::CoralGround:
        {
            elevatorHeight  = CoralPoseConstants::GroundElevator;
            armAngle        = CoralPoseConstants::GroundArmAngle;
            wristAngle      = CoralPoseConstants::GroundWristAngle;
            gripperVoltage  = CoralPoseConstants::GroundGripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralStation:
        {
            elevatorHeight  = CoralPoseConstants::StationElevator;
            armAngle        = CoralPoseConstants::StationArmAngle;
            wristAngle      = CoralPoseConstants::StationWristAngle;
            gripperVoltage  = CoralPoseConstants::StationGripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralL1:
        {
            elevatorHeight  = CoralPoseConstants::L1Elevator;
            armAngle        = CoralPoseConstants::L1ArmAngle;
            wristAngle      = CoralPoseConstants::L1WristAngle;
            gripperVoltage  = CoralPoseConstants::L1GripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralL2:
        {
            elevatorHeight  = CoralPoseConstants::L2Elevator;
            armAngle        = CoralPoseConstants::L2ArmAngle;
            wristAngle      = CoralPoseConstants::L2WristAngle;
            gripperVoltage  = CoralPoseConstants::L2GripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralL3:
        {
            elevatorHeight  = CoralPoseConstants::L3Elevator;
            armAngle        = CoralPoseConstants::L3ArmAngle;
            wristAngle      = CoralPoseConstants::L3WristAngle;
            gripperVoltage  = CoralPoseConstants::L3GripperVoltage;
            break;
        }

        case GripperPoseEnum::CoralL4:
        {
            elevatorHeight  = CoralPoseConstants::L4Elevator;
            armAngle        = CoralPoseConstants::L4ArmAngle;
            wristAngle      = CoralPoseConstants::L4WristAngle;
            gripperVoltage  = CoralPoseConstants::L4GripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeGround:
        {
            elevatorHeight  = AlgaePoseConstants::GroundElevator;
            armAngle        = AlgaePoseConstants::GroundArmAngle;
            wristAngle      = AlgaePoseConstants::GroundWristAngle;
            gripperVoltage  = AlgaePoseConstants::GroundGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeOnCoral:
        {
            elevatorHeight  = AlgaePoseConstants::OnCoralElevator;
            armAngle        = AlgaePoseConstants::OnCoralArmAngle;
            wristAngle      = AlgaePoseConstants::OnCoralWristAngle;
            gripperVoltage  = AlgaePoseConstants::OnCoralGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeLo:
        {
            elevatorHeight  = AlgaePoseConstants::LoElevator;
            armAngle        = AlgaePoseConstants::LoArmAngle;
            wristAngle      = AlgaePoseConstants::LoWristAngle;
            gripperVoltage  = AlgaePoseConstants::LoGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeHigh:
        {
            elevatorHeight  = AlgaePoseConstants::HighElevator;
            armAngle        = AlgaePoseConstants::HighArmAngle;
            wristAngle      = AlgaePoseConstants::HighWristAngle;
            gripperVoltage  = AlgaePoseConstants::HighGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeProcessor:
        {
            elevatorHeight  = AlgaePoseConstants::ProcessorElevator;
            armAngle        = AlgaePoseConstants::ProcessorArmAngle;
            wristAngle      = AlgaePoseConstants::ProcessorWristAngle;
            gripperVoltage  = AlgaePoseConstants::ProcessorGripperVoltage;
            break;
        }

        case GripperPoseEnum::AlgaeBarge:
        {
            elevatorHeight  = AlgaePoseConstants::BargeElevator;
            armAngle        = AlgaePoseConstants::BargeArmAngle;
            wristAngle      = AlgaePoseConstants::BargeWristAngle;
            gripperVoltage  = AlgaePoseConstants::BargeGripperVoltage;
            break;
        }
    }

    // Remember the pose
    m_pose = pose;

    // Set the elevator height
    SetElevatorHeight(elevatorHeight);

    // Set the arm angle
    SetArmAngle(armAngle);

    // Set the wrist angle
    SetWristAngle(wristAngle);

    // Set the gripper wheels voltage
    SetGripperWheelsVoltage(gripperVoltage);
}
#pragma endregion

#pragma region SetElevatorHeight
/// @brief Method to set the elevator height.
/// @param position The setpoint for the elevator height.
void Gripper::SetElevatorHeight(units::length::meter_t position)
{
    // Compute the number of turns based on the specficied position
    units::angle::turn_t newPosition = (units::angle::turn_t) (position.value() * ElevatorConstants::PositionToTurnsConversionFactor);

    // Set the elevator set position
    m_elevatorMotor->SetControl(m_elevatorMotionMagicVoltage.WithPosition(newPosition).WithSlot(0));
}
#pragma endregion

#pragma region SetElevatorOffset
/// @brief Moves the elevator by the given offset
/// @param offset The Given offset
void Gripper::SetElevatorOffset(units::length::meter_t offset)
{
    units::length::meter_t position = (units::length::meter_t)(m_elevatorMotor->GetPosition().GetValue().value() /
                        ElevatorConstants::PositionToTurnsConversionFactor) + offset;
    SetElevatorHeight(position);
}
#pragma endregion

#pragma region SetArmAngleTalon
/// @brief Method to set the arm angle.
/// @param position The setpoint for the arm angle. Takes -180 -> 180
void Gripper::SetArmAngleTalon(units::angle::degree_t angle)
{
    // // Making sure that the climb doesn't try to go through the robot
    // if (angle < ArmConstants::MinimumPosition)  // TODO: Need to calibrate angle to motor rotations
    //    angle = ArmConstants::MinimumPosition;

    // if (angle > ArmConstants::MaximumPosition)
    //     angle = ArmConstants::MaximumPosition;

    // Compute the number of turns based on the specficied angle
    units::angle::turn_t newPosition = (units::angle::turn_t) (angle.value() * ArmConstantsTalon::AngleToTurnsConversionFactor.value());

    // Set the arm set position
    m_armMotorTalon->SetControl(m_motionMagicVoltage.WithPosition(newPosition).WithSlot(0));
}
#pragma endregion

#pragma region GetArmAngleTalon
/// @brief Method to get the arm angle.
/// @return The arm angle.
units::angle::degree_t Gripper::GetArmAngleTalon()
{
    // Get the current arm motor angle
    auto currentAngle = m_armMotorTalon->GetPosition().GetValueAsDouble();

    // Return the arm angle
    return (units::angle::degree_t) (currentAngle /ArmConstantsTalon::AngleToTurnsConversionFactor.value());
}
#pragma endregion

#pragma region SetArmAngle
/// @brief Method to set the Wrist angle.
/// @param position The setpoint for the Wrist angle.
void Gripper::SetArmAngle(units::angle::degree_t angle)
{
    // Convert the position to radians
    double positionRadian = (angle.value() * std::numbers::pi) / 180.0;

    // Set the Wrist set position
    m_armTurnClosedLoopController.SetReference(positionRadian, rev::spark::SparkMax::ControlType::kPosition);
}
#pragma endregion

#pragma region GetArmAngle
/// @brief Method to get the arm angle.
/// @return The arm angle.
units::angle::degree_t Gripper::GetArmAngle()
{
    // Get the current arm motor angle
    double angle = m_armEncoder.GetPosition() * (180.0 / std::numbers::pi);

    // Return the arm angle
    return units::angle::degree_t{angle};
}
#pragma endregion

#pragma region SetArmAngleOffset
/// @brief Method to set the arm angle.
/// @param position The setpoint for the arm angle. Takes -180 -> 180
void Gripper::SetArmAngleOffset(units::angle::degree_t offset)
{
    // Set the arm angle based on the offset
    SetArmAngle(GetArmAngleTalon() + offset);
}
#pragma endregion

#pragma region SetWristAngle
/// @brief Method to set the Wrist angle.
/// @param position The setpoint for the Wrist angle.
void Gripper::SetWristAngle(units::angle::degree_t angle)
{
    // Convert the position to radians
    double positionRadian = (angle.value() * std::numbers::pi) / 180.0;

    // Set the Wrist set position
    m_wristTurnClosedLoopController.SetReference(positionRadian, rev::spark::SparkMax::ControlType::kPosition);
}
#pragma endregion

#pragma region GetWristAngle
/// @brief Method to get the arm angle.
/// @return The arm angle.
units::angle::degree_t Gripper::GetWristAngle()
{
    // Get the current arm motor angle
    double angle = m_wristEncoder.GetPosition() * (180.0 / std::numbers::pi);

    // Return the arm angle
    return units::angle::degree_t{angle};
}
#pragma endregion

#pragma region SetGripperWheelsVoltage
/// @brief Method to set the Gripper wheels voltage.
/// @param voltage The setpoint for the Gripper wheels voltage.
void Gripper::SetGripperWheelsVoltage(units::voltage::volt_t voltage)
{
    // Set the voltage of the Gripper wheels
    m_gripperMotorRight.SetVoltage(voltage);
    m_gripperMotorLeft.SetVoltage(voltage);
}
#pragma endregion
