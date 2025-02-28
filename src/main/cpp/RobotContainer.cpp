#include "RobotContainer.h"

// Reference to the RobotContainer singleton class
RobotContainer *RobotContainer::m_robotContainer = NULL;

#pragma region GetInstance
/// @brief Method to return a pointer to the RobotContainer class.
/// @return Pointer to the RobotContainer class.
RobotContainer *RobotContainer::GetInstance()
{
    // Detrermine if the class has already been instantiated
    if (m_robotContainer == NULL)
    {
        // Instantiate the class
        m_robotContainer = new RobotContainer();
    }

    // Return the class pointer
    return m_robotContainer;
}
#pragma endregion

#pragma region RobotContainer
/// @brief Method to configure the robot and SmartDashboard configuration.
RobotContainer::RobotContainer()
{
    // Bind the joystick controls to the robot commands
    ConfigureButtonBindings();

    frc::SmartDashboard::PutData("Chassis: Time ",           new ChassisDriveTime(2_s, 0.5_mps,                                           &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: OneMeter",        new ChassisDrivePose(2.0_mps, 1_m,  0_m,  90_deg,        10_s,               &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: TwoMeters",       new ChassisDrivePose(2.0_mps, 2_m,  2_m,   0_deg,        10_s,               &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: Turn ",           new ChassisDrivePose(2.0_mps, 0_m,  0_m,  45_deg,        10_s,               &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: AprilTag ",       new ChassisDriveToAprilTag(1.0_mps, 0.0_m, 0.0_m, 0_deg, 10_s, &m_aprilTags, &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: Serpentine ",     new ChassisDriveSerpentine(1.0_mps,                      10_s,               &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: Drive to Wall ",  new ChassisDriveToWall(1.0_mps,     1_m,                 10_s,               &m_drivetrain));

    frc::SmartDashboard::PutData("Elevator Jog Up",          new frc2::InstantCommand([this] { m_gripper.SetElevatorOffset( ElevatorConstants::HeightOffset); }));
    frc::SmartDashboard::PutData("Elevator Jog Down",        new frc2::InstantCommand([this] { m_gripper.SetElevatorOffset(-ElevatorConstants::HeightOffset); }));

    frc::SmartDashboard::PutData("Arm Jog Positive",         new frc2::InstantCommand([this] { m_gripper.SetArmAngleOffset( ArmConstants::AngleOffset);}));
    frc::SmartDashboard::PutData("Arm Jog Negative",         new frc2::InstantCommand([this] { m_gripper.SetArmAngleOffset(-ArmConstants::AngleOffset);}));

    frc::SmartDashboard::PutData("Wrist Jog Positive",       new frc2::InstantCommand([this] { m_gripper.SetWristAngleOffset( WristConstants::AngleOffset);}));
    frc::SmartDashboard::PutData("Wrist Jog Negative",       new frc2::InstantCommand([this] { m_gripper.SetWristAngleOffset(-WristConstants::AngleOffset);}));

    frc::SmartDashboard::PutData("Coral: Ground",            new GripperPose(GripperPoseEnum::CoralGround,    &m_gripper));
    frc::SmartDashboard::PutData("Coral: Station",           new GripperPose(GripperPoseEnum::CoralStation,   &m_gripper));
    frc::SmartDashboard::PutData("Coral: L1",                new GripperPose(GripperPoseEnum::CoralL1,        &m_gripper));
    frc::SmartDashboard::PutData("Coral: L2",                new GripperPose(GripperPoseEnum::CoralL2,        &m_gripper));
    frc::SmartDashboard::PutData("Coral: L3",                new GripperPose(GripperPoseEnum::CoralL3,        &m_gripper));
    frc::SmartDashboard::PutData("Coral: L4",                new GripperPose(GripperPoseEnum::CoralL4,        &m_gripper));

    frc::SmartDashboard::PutData("Algae: Ground",            new GripperPose(GripperPoseEnum::AlgaeGround,    &m_gripper));
    frc::SmartDashboard::PutData("Algae: Coral",             new GripperPose(GripperPoseEnum::AlgaeOnCoral,   &m_gripper));
    frc::SmartDashboard::PutData("Algae: Low",               new GripperPose(GripperPoseEnum::AlgaeLow,       &m_gripper));
    frc::SmartDashboard::PutData("Algae: High",              new GripperPose(GripperPoseEnum::AlgaeHigh,      &m_gripper));
    frc::SmartDashboard::PutData("Algae: Processor",         new GripperPose(GripperPoseEnum::AlgaeProcessor, &m_gripper));
    frc::SmartDashboard::PutData("Algae: Barge",             new GripperPose(GripperPoseEnum::AlgaeBarge,     &m_gripper));

    frc::SmartDashboard::PutData("Gripper: Activate",        new GripperActivate(&m_gripper));

    // Configure the autonomous command chooser
    m_autonomousChooser.SetDefaultOption("Do Nothing",       new AutonomousDoNothing());
    m_autonomousChooser.AddOption("Drive Forward OneMeter",  new ChassisDrivePose(1.0_mps, 1_m, 0_m, 0_deg, 10_s, &m_drivetrain));
    m_autonomousChooser.AddOption("Drive Forward TwoMeters", new ChassisDrivePose(1.0_mps, 2_m, 0_m, 0_deg, 10_s, &m_drivetrain));
    m_autonomousChooser.AddOption("Led Autonomous",          new AutonomousLed(&m_leds));
    m_autonomousChooser.AddOption("Parallel Test",           new AutonomousParallel(&m_leds, &m_drivetrain));
    m_autonomousChooser.AddOption("Complex Test",            new AutonomousComplex(&m_leds,  &m_drivetrain));

    // Send the autonomous mode chooser to the SmartDashboard
    frc::SmartDashboard::PutData("Autonomous Mode", &m_autonomousChooser);

    m_startingPositionChooser.SetDefaultOption("Middle", "M");
    m_startingPositionChooser.AddOption("Left",          "L");
    m_startingPositionChooser.AddOption("Right",         "R");

    frc::SmartDashboard::PutData("Start Position", &m_startingPositionChooser);

    // Set the default commands for the subsystems
    m_drivetrain.SetDefaultCommand(ChassisDrive([this] { return Forward(); },
                                                [this] { return Strafe();  },
                                                [this] { return Angle();   },
                                                &m_drivetrain));

    // Set the LED default command
    m_leds.SetDefaultCommand(SetLeds(LedMode::Off, &m_leds));

    // Set the swerve wheels to zero
    SetSwerveWheelAnglesToZero();
}
#pragma endregion

#pragma region ConfigureButtonBindings
/// @brief Method to bind the joystick controls to the robot commands.
void RobotContainer::ConfigureButtonBindings()
{
    // Configure the driver controls
    ConfigureDriverControls();

    // Configure the operator controls
    ConfigureCoralPoseControls();
    ConfigureAlgaePoseControls();
    ConfigureGripperControls();
    ConfigureClimberControls();

    // Scores/Intakes Algae/Coral
    frc2::JoystickButton (&m_operatorController, ControlPanelConstants::Activate)
        .OnTrue(GripperActivate(&m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
}
#pragma endregion

#pragma region ConfigureDriverControls
/// @brief Method to bind the driver joystick controls to the robot commands.
void RobotContainer::ConfigureDriverControls()
{
    // Drive to position using the AprilTag

    // Use the trigger to activate the operation

    // L1Score.OnTrue(AprilTagScoreCoral(GripperPoseEnum::CoralL1,
    //                                   [this]()->bool { return m_operatorController.GetRawButtonPressed(ControlPanelConstants::CoralSideSelect);},
    //                                   &m_aprilTags, &m_gripper, &m_drivetrain).ToPtr());

    // Reset the gyro angle
    frc2::JoystickButton (&m_driverController, Extreme3DConstants::HandleUpperLeft)
        .OnTrue(new frc2::InstantCommand([this] { m_drivetrain.ZeroHeading(); }, {&m_drivetrain}));

    // Set field centricity on
    frc2::JoystickButton (&m_driverController, Extreme3DConstants::HandleLowerLeft)
        .OnTrue(new frc2::InstantCommand([this] { m_drivetrain.SetFieldCentricity(true); }, {&m_drivetrain}));

    // Set field centricity off
    frc2::JoystickButton (&m_driverController, Extreme3DConstants::HandleLowerRight)
        .OnTrue(new frc2::InstantCommand([this] { m_drivetrain.SetFieldCentricity(false); }, {&m_drivetrain}));

    // Toggle X mode
    frc2::JoystickButton (&m_driverController, frc::XboxController::Button::kX)
        .WhileTrue(new frc2::RunCommand([this] { m_drivetrain.SetX(); }, {&m_drivetrain}));
}
#pragma endregion

#pragma region ConfigureCoralPoseControls
/// @brief Method to bind the operator control panel scoring/intaking positioning, then pressing activate (ex: L1Score then Activate).
void RobotContainer::ConfigureCoralPoseControls()
{
    // Define an array of button mappings for coral poses
    struct CoralPoseMapping
    {
        int             button;
        GripperPoseEnum pose;
    };

    CoralPoseMapping coralPoses[] =
    {
        {ControlPanelConstants::CoralGnd, GripperPoseEnum::CoralGround},
        {ControlPanelConstants::CoralStn, GripperPoseEnum::CoralStation},
        {ControlPanelConstants::CoralL1,  GripperPoseEnum::CoralL1},
        {ControlPanelConstants::CoralL2,  GripperPoseEnum::CoralL2},
        {ControlPanelConstants::CoralL3,  GripperPoseEnum::CoralL3},
        {ControlPanelConstants::CoralL4,  GripperPoseEnum::CoralL4}
    };

    // Iterate through the array and bind the buttons to the corresponding poses
    for (const auto& mapping : coralPoses)
    {
        frc2::JoystickButton (&m_operatorController, mapping.button)
            .OnTrue(GripperPose(mapping.pose, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    }
}
#pragma endregion

#pragma region ConfigureAlgaePoseControls
/// @brief Method to bind the operator control panel scoring/intaking positioning, then pressing activate (ex: L1Score then Activate).
void RobotContainer::ConfigureAlgaePoseControls()
{
    // Define an array of button mappings for algae poses
    struct AlgaePoseMapping
    {
        int             button;
        GripperPoseEnum pose;
    };

    AlgaePoseMapping algaePoses[] =
    {
        {ControlPanelConstants::AlgaeGnd,       GripperPoseEnum::AlgaeGround},
        {ControlPanelConstants::AlgaeCoral,     GripperPoseEnum::AlgaeOnCoral},
        {ControlPanelConstants::AlgaeLow,       GripperPoseEnum::AlgaeLow},
        {ControlPanelConstants::AlgaeHigh,      GripperPoseEnum::AlgaeHigh},
        {ControlPanelConstants::AlgaeProcessor, GripperPoseEnum::AlgaeProcessor},
        {ControlPanelConstants::AlgaeBarge,     GripperPoseEnum::AlgaeBarge}
    };

    // Iterate through the array and bind the buttons to the corresponding poses
    for (const auto& mapping : algaePoses)
    {
        frc2::JoystickButton (&m_operatorController, mapping.button)
            .OnTrue(GripperPose(mapping.pose, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    }
}
#pragma endregion

#pragma region ConfigureGripperControls
/// @brief Method to bind the operator control panel gripper controls.
void RobotContainer::ConfigureGripperControls()
{
    // Manually offsets elevator upwards
    frc2::JoystickButton (&m_operatorController, ControlPanelConstants::ElevatorUp)
        .OnTrue(new frc2::RunCommand([this] { m_gripper.SetElevatorOffset(ElevatorConstants::HeightOffset);}));

    // Manually offsets elevator downwards
    frc2::JoystickButton (&m_operatorController, ControlPanelConstants::ElevatorDown)
        .OnTrue(new frc2::RunCommand([this] { m_gripper.SetElevatorOffset(-ElevatorConstants::HeightOffset);}));
}
#pragma endregion

#pragma region ConfigureClimberControls
/// @brief Method to bind the operator control panel climb controls.
void RobotContainer::ConfigureClimberControls()
{
    // Manually offsets climb upwards
    frc2::JoystickButton (&m_operatorController, ControlPanelConstants::ClimbUp)
        .WhileTrue(new frc2::RunCommand([this] { m_climb.SetVoltage(ClimbConstants::ClimbVoltage); }, {&m_climb}))
        .OnFalse(new frc2::InstantCommand([this] { m_climb.SetVoltage(0_V); }, {&m_climb}));

    // Manually offsets climb downwards
    frc2::JoystickButton (&m_operatorController, ControlPanelConstants::ClimbDown)
        .WhileTrue(new frc2::RunCommand([this] { m_climb.SetVoltage(-ClimbConstants::ClimbVoltage); }, {&m_climb}))
        .OnFalse(new frc2::InstantCommand([this] { m_climb.SetVoltage(0_V); }, {&m_climb}));
}
#pragma endregion

#pragma region GetDriverController
/// @brief Method to return a pointer to the driver joystick.
/// @return Pointer to the driver joystick.
frc::Joystick *RobotContainer::GetDriverController()
{
    // Return the pointer to the driver joystick
    return &m_driverController;
}
#pragma endregion

#pragma region GetOperatorController
/// @brief Method to return a pointer to the controller joystick.
/// @return Pointer to the controller joystick.
frc::XboxController *RobotContainer::GetOperatorController()
{
    // Return the pointer to the operator joystick
    return &m_operatorController;
}
#pragma endregion

#pragma region GetAutonomousCommand
/// @brief Method to return a pointer to the autonomous command.
/// @return Pointer to the autonomous command
frc2::Command *RobotContainer::GetAutonomousCommand()
{
    // The selected command will be run in autonomous
    return m_autonomousChooser.GetSelected();
}
#pragma endregion

#pragma region GetStartPosition
/// @brief Method to get the starting position for the robot.
/// @return String representing the starting position.
std::string RobotContainer::GetStartPosition()
{
    // Return the selected starting position
    return m_startingPositionChooser.GetSelected();
}
#pragma endregion

#pragma region SetSwerveWheelAnglesToZero
 /// @brief Method to set the swerve wheels to starting position based on the absolute encoder.
 void RobotContainer::SetSwerveWheelAnglesToZero()
 {
    // Execute the command
    m_swerveWheelAnglesToZero->Execute();
 }
#pragma endregion

#pragma region Forward
/// @brief Method to return the forward joystick value.
/// @return The forward joystick meters per second value.
units::meters_per_second_t RobotContainer::Forward()
{
    // Get the forward joystick setting
    double joystickForward = GetDriverController()->GetRawAxis(ControllerConstants::JoystickForwardIndex);

    // Use exponential function to calculate the forward value for better slow speed control
    joystickForward = GetExponentialValue(joystickForward, ControllerConstants::ExponentForward);

    // Return the x speed
    return -m_xspeedLimiter.Calculate(frc::ApplyDeadband(joystickForward, ControllerConstants::JoystickDeadZone)) * DrivetrainConstants::MaxSpeed;
}
#pragma endregion

#pragma region Strafe
/// @brief Method to return the strafe joystick value.
/// @return The strafe joystick meters per second value.
units::meters_per_second_t RobotContainer::Strafe()
{
    // Get the strafe joystick setting
    double joystickStrafe = GetDriverController()->GetRawAxis(ControllerConstants::JoystickStrafeIndex);

    // Use exponential function to calculate the forward value for better slow speed control
    joystickStrafe = GetExponentialValue(joystickStrafe, ControllerConstants::ExponentStrafe);

    // Return the y speed
    return -m_yspeedLimiter.Calculate(frc::ApplyDeadband(joystickStrafe, ControllerConstants::JoystickDeadZone)) * DrivetrainConstants::MaxSpeed;
}
#pragma endregion

#pragma region Angle
/// @brief Method to return the angle joystick value.
/// @return The angle joystick value.
units::radians_per_second_t RobotContainer::Angle()
{
    // Get the angle joystick setting
    double joystickAngle = GetDriverController()->GetRawAxis(ControllerConstants::JoystickAngleIndex);

    // Apply deadband first
    double deadbandedAngle = frc::ApplyDeadband(joystickAngle, ControllerConstants::JoystickRotateDeadZone);

    // Use exponential function to calculate the angle value for better slow speed control
    double exponentialAngle = GetExponentialValue(deadbandedAngle, ControllerConstants::ExponentAngle);

    // Apply smoothing between frames to reduce jerky movement (inline implementation)
    // Smoothing factor: 0.0-1.0 (higher = more smoothing, 0.3 is a good starting point)
    constexpr double kSmoothingFactor   = 0.3;
    static double    previousAngleInput = 0.0; // Static variable persists between function calls

    // Calculate smoothed value using previous output and current input
    double smoothedAngle = kSmoothingFactor * previousAngleInput + (1.0 - kSmoothingFactor) * exponentialAngle;
    previousAngleInput   = smoothedAngle; // Store for next cycle

    // Return the rotation speed with rate limiter applied
    return units::radians_per_second_t(-m_rotLimiter.Calculate(smoothedAngle) * DrivetrainConstants::MaxAngularSpeed);
}
#pragma endregion

#pragma region ExponentialValue
/// @brief Method to convert a joystick value from -1.0 to 1.0 to exponential mode.
/// @param joystickValue The raw joystick value.
/// @param exponent The exponential value.
/// @return The resultant exponential value.
double RobotContainer::GetExponentialValue(double joystickValue, double exponent)
{
    int    direction = (joystickValue < 0.0) ? -1 : 1;
    double absValue  = std::abs(joystickValue);
    double output    = std::pow(absValue, exponent) * direction;

    // Ensure the range of the output
    if (output < -1.0) output = -1.0;
    if (output > 1.0)  output = 1.0;

    // Return the output value
    return output;
}
#pragma endregion

#pragma region GetChassisPose
/// @brief Method to get the chassis Pose.
/// @return The chassis Pose.
frc::Pose2d RobotContainer::GetChassisPose()
{
    // Return the chassis pose
    return m_drivetrain.GetPose();
}
#pragma endregion

#pragma region GetGripper
/// @brief Method to return a pointer to the gripper subsystem.
/// @return Pointer to the gripper subsystem.
Gripper *RobotContainer::GetGripper()
{
    // Return the pointer to the gripper
    return &m_gripper;
}
#pragma endregion

#pragma region GetPowerDistribution
/// @brief Method to return a pointer to the power distribution panel.
frc::PowerDistribution *RobotContainer::GetPowerDistribution()
{
    // Return the pointer to the power distribution panel
    return &m_powerDistribution;
}
#pragma endregion
