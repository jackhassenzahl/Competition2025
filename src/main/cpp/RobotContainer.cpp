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
    frc::SmartDashboard::PutData("Chassis: Time ",          new ChassisDriveTime(2_s, 0.5_mps,                                           &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: OneMeter",       new ChassisDrivePose(2.0_mps, 1_m,  0_m,  90_deg,        10_s,               &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: TwoMeters",      new ChassisDrivePose(2.0_mps, 2_m,  2_m,   0_deg,        10_s,               &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: Turn ",          new ChassisDrivePose(2.0_mps, 0_m,  0_m,  45_deg,        10_s,               &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: AprilTag ",      new ChassisDriveToAprilTag(1.0_mps, 0.0_m, 0.0_m, 0_deg, 10_s, &m_aprilTags, &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: Serpentine ",    new ChassisDriveSerpentine(1.0_mps,                      10_s,               &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: Drive to Wall ", new ChassisDriveToWall(1.0_mps,     1_m,                 10_s,               &m_drivetrain));

    frc::SmartDashboard::PutData("Coral: Ground",           new GripperPose(GripperPoseEnum::CoralGround, &m_gripper));
    frc::SmartDashboard::PutData("Coral: L1",               new GripperPose(GripperPoseEnum::CoralL1,     &m_gripper));
    frc::SmartDashboard::PutData("Coral: L2",               new GripperPose(GripperPoseEnum::CoralL2,     &m_gripper));
    frc::SmartDashboard::PutData("Coral: L3",               new GripperPose(GripperPoseEnum::CoralL3,     &m_gripper));
    frc::SmartDashboard::PutData("Coral: L4",               new GripperPose(GripperPoseEnum::CoralL4,     &m_gripper));

    // Bind the joystick controls to the robot commands
    ConfigureButtonBindings();

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
    m_drivetrain.SetDefaultCommand(ChassisDrive(
        [this] { return Forward(); },
        [this] { return Strafe();  },
        [this] { return Angle();   },
        &m_drivetrain));

    m_leds.SetDefaultCommand(SetLeds(LedMode::Off, &m_leds));

    // Set the swerve wheels to zero
    SetSwerveWheelAnglesToZero();
}
#pragma endregion

#pragma region ConfigureButtonBindings
/// @brief Method to bind the joystick controls to the robot commands.
void RobotContainer::ConfigureButtonBindings()
{
    /**************************** Driver Buttons ***********************************************/

    // Toggle Field Centricity On
    frc2::JoystickButton fieldCentricOn(&m_driverController, Extreme3DConstants::HandleLowerLeft);
    fieldCentricOn.OnTrue(ChassisSetFieldCentricity(true, &m_drivetrain).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Toggle Field Centricity On
    frc2::JoystickButton fieldCentricOff(&m_driverController, Extreme3DConstants::HandleLowerRight);
    fieldCentricOff.OnTrue(ChassisSetFieldCentricity(false, &m_drivetrain).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Toggle X mode
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX).WhileTrue(new frc2::RunCommand([this] { m_drivetrain.SetX(); }, {&m_drivetrain}));

    /**************************** Operator Buttons - Chassis Pose ******************************/
    // Scoring/Intaking requires positioning, then pressing activate (ex: L1Score then Activate)

    // Elevator/Arm to L1
    frc2::JoystickButton L1(&m_operatorController, XBoxConstants::A);
    L1.OnTrue(GripperPose(GripperPoseEnum::CoralL1, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Elevator/Arm to L2
    frc2::JoystickButton L2(&m_operatorController, XBoxConstants::B);
    L2.OnTrue(GripperPose(GripperPoseEnum::CoralL2, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Elevator/Arm to L3
    frc2::JoystickButton L3(&m_operatorController, XBoxConstants::X);
    L3.OnTrue(GripperPose(GripperPoseEnum::CoralL3, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Elevator/Arm to L4
    frc2::JoystickButton L4(&m_operatorController, XBoxConstants::Y);
    L4.OnTrue(GripperPose(GripperPoseEnum::CoralL4, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Move to, and ready to score L1 (does not score)
    frc2::JoystickButton L1Score(&m_operatorController, ControlPanelConstants::CoralL1);
    L1Score.OnTrue(AutonomusScoreCoral(
        GripperPoseEnum::CoralL1,
        [this]()->bool { return m_operatorController.GetRawButtonPressed(ControlPanelConstants::CoralSideSelect);},
        &m_aprilTags,
        &m_gripper,
        &m_drivetrain
    ).ToPtr());

    // Move to, and ready to score L2 (does not score)
    frc2::JoystickButton L2Score(&m_operatorController, ControlPanelConstants::CoralL2);
    L2Score.OnTrue(AutonomusScoreCoral(
        GripperPoseEnum::CoralL2,
        [this]()->bool { return m_operatorController.GetRawButtonPressed(ControlPanelConstants::CoralSideSelect);},
        &m_aprilTags,
        &m_gripper,
        &m_drivetrain
    ).ToPtr());

    // Move to, and ready to score L3 (does not score)
    frc2::JoystickButton L3Score(&m_operatorController, ControlPanelConstants::CoralL3);
    L3Score.OnTrue(AutonomusScoreCoral(
        GripperPoseEnum::CoralL3,
        [this]()->bool { return m_operatorController.GetRawButtonPressed(ControlPanelConstants::CoralSideSelect);},
        &m_aprilTags,
        &m_gripper,
        &m_drivetrain
    ).ToPtr());

    // Move to, and ready to score L4 (does not score)
    frc2::JoystickButton L4Score(&m_operatorController, ControlPanelConstants::CoralL4);
    L4Score.OnTrue(AutonomusScoreCoral(
        GripperPoseEnum::CoralL4,
        [this]()->bool { return m_operatorController.GetRawButtonPressed(ControlPanelConstants::CoralSideSelect);},
        &m_aprilTags,
        &m_gripper,
        &m_drivetrain
    ).ToPtr());

    // Scores/Intakes Algae/Coral
    frc2::JoystickButton Activate(&m_operatorController, ControlPanelConstants::Activate);
    Activate.OnTrue(GripperActivate(&m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Positions to intake coral from ground (does not intake)
    frc2::JoystickButton CoralGround(&m_operatorController, ControlPanelConstants::CoralGnd);
    CoralGround.OnTrue(GripperPose(GripperPoseEnum::CoralGround, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Positions to intake coral from station (does not intake)
    frc2::JoystickButton CoralStation(&m_operatorController, ControlPanelConstants::CoralStn);
    CoralStation.OnTrue(GripperPose(GripperPoseEnum::CoralStation, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Positions to intake algae from ground (does not intake)
    frc2::JoystickButton AlgaeGround(&m_operatorController, ControlPanelConstants::AlgaeGnd);
    AlgaeGround.OnTrue(GripperPose(GripperPoseEnum::AlgaeGround, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Positions to intake algae from on top of coral (does not intake)
    frc2::JoystickButton AlgaeOnCoral(&m_operatorController, ControlPanelConstants::AlgaeCoral);
    AlgaeOnCoral.OnTrue(GripperPose(GripperPoseEnum::AlgaeOnCoral, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Positions to intake algae from between L2 and L3 on the reef
    frc2::JoystickButton AlgaeLo(&m_operatorController, ControlPanelConstants::AlgaeLo);
    AlgaeLo.OnTrue(GripperPose(GripperPoseEnum::AlgaeLo, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Positions to intake algae from between L3 and L4 on the reef
    frc2::JoystickButton AlgaeHigh(&m_operatorController, ControlPanelConstants::AlgaeHi);
    AlgaeHigh.OnTrue(GripperPose(GripperPoseEnum::AlgaeLo, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Positions to score algae into processor
    frc2::JoystickButton AlageProcessor(&m_operatorController, ControlPanelConstants::AlgaeProcessor);
    AlageProcessor.OnTrue(GripperPose(GripperPoseEnum::AlgaeLo, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Positions to score algae onto barge
    frc2::JoystickButton AlgaeBarge(&m_operatorController, ControlPanelConstants::AlgaeBarge);
    AlgaeBarge.OnTrue(GripperPose(GripperPoseEnum::AlgaeLo, &m_gripper).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Manually offsets elevator upwards
    frc2::JoystickButton elevatorUp(&m_operatorController, ControlPanelConstants::ElevatorUp);
    elevatorUp.OnTrue(new frc2::RunCommand([this] { m_gripper.SetElevatorOffset(ElevatorConstants::HeightOffset);}));

    // Manually offsets elevator downwards
    frc2::JoystickButton elevatorDown(&m_operatorController, ControlPanelConstants::ElevatorDown);
    elevatorDown.OnTrue(new frc2::RunCommand([this] { m_gripper.SetElevatorOffset(-ElevatorConstants::HeightOffset);}));

    // Manually offsets climb upwards
    frc2::JoystickButton climbUp(&m_operatorController, XBoxConstants::RightBumper);
    climbUp.WhileTrue(new frc2::RunCommand([this] { m_climb.SetVoltage(ClimbConstants::ClimbVoltage); }, {&m_climb}))
           .OnFalse(new frc2::InstantCommand([this] { m_climb.SetVoltage(0_V); }, {&m_climb}));

    // Manually offsets climb downwards
    frc2::JoystickButton climbDown(&m_operatorController, XBoxConstants::LeftBumper);
    climbDown.WhileTrue(new frc2::RunCommand([this] { m_climb.SetVoltage(-ClimbConstants::ClimbVoltage); }, {&m_climb}))
             .OnFalse(new frc2::InstantCommand([this] { m_climb.SetVoltage(0_V); }, {&m_climb}));

    /**************************** Operator Buttons - LEDs **************************************/

    frc2::JoystickButton setLedsOff(&m_operatorController, XBoxConstants::LeftStickButton);
    setLedsOff.OnTrue(SetLeds(LedMode::Off, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::JoystickButton setLedsStrobe(&m_operatorController, XBoxConstants::RightStickButton);
    setLedsStrobe.OnTrue(SetLeds(LedMode::Strobe, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::JoystickButton setLedsShootingAnimation{&m_operatorController, XBoxConstants::Y};
    setLedsShootingAnimation.OnTrue(SetLeds(LedMode::ShootingAnimation, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::POVButton setLedsSolidGreen{&m_operatorController, XBoxConstants::Pov_0};
    setLedsSolidGreen.OnTrue(SetLeds(LedMode::SolidGreen, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::POVButton setLedsSolidRed{&m_operatorController, XBoxConstants::Pov_90};
    setLedsSolidRed.OnTrue(SetLeds(LedMode::SolidRed, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::POVButton setLedsHvaColors{&m_operatorController, XBoxConstants::Pov_180};
    setLedsHvaColors.OnTrue(SetLeds(LedMode::HvaColors, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::POVButton setLedsRainbow{&m_operatorController, XBoxConstants::Pov_270};
    setLedsRainbow.OnTrue(SetLeds(LedMode::Rainbow, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
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
    previousAngleInput = smoothedAngle; // Store for next cycle

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

#pragma region SetPeriod
/// @brief Method to set the timed robot period.
/// @param period The period to set.
void RobotContainer::SetPeriod(units::second_t period)
{
    // Set the period
    m_period = period;
}
#pragma endregion

#pragma region GetPeriod
/// @brief Method to get the timed robot period.
/// @return The timed robot period.
units::second_t RobotContainer::GetPeriod()
{
    // Return the timed robot period
    return m_period;
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
