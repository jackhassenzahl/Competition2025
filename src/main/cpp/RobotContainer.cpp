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
    frc::SmartDashboard::PutData("Chassis: Stop",       new ChassisDriveDistance(0_m,     0_mps,   0_s,               &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: Time ",      new ChassisDriveTime(2_s,         0.5_mps,                    &m_drivetrain));   
    frc::SmartDashboard::PutData("Chassis: OneMeter",   new ChassisDriveDistance(1_m,     0.5_mps, 5_s,               &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: TwoMeters",  new ChassisDriveDistance(2_m,     0.5_mps, 5_s,               &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: Turn ",      new ChassisDriveTurnAngle(45_deg, 0.5_mps, 5_s,               &m_drivetrain));
    frc::SmartDashboard::PutData("Chassis: AprilTag ",  new ChassisDriveToAprilTag(       0.5_mps, 5_s, &m_aprilTags, &m_drivetrain));

    frc::SmartDashboard::PutData("Elevator: Zero",      new ElevatorSetHeight(0_m, &m_elevator));
    frc::SmartDashboard::PutData("Elevator: OneMeter",  new ElevatorSetHeight(1_m, &m_elevator));
       
    // Bind the joystick controls to the robot commands
    ConfigureButtonBindings();

    // Configure the autonomous command chooser
    m_autonomousChooser.SetDefaultOption("Do Nothing",       new AutonomousDoNothing());
    m_autonomousChooser.AddOption("Drive Forward OneMeter",  new ChassisDriveDistance(1_m, 0.5_mps, 5_s, &m_drivetrain));
    m_autonomousChooser.AddOption("Drive Forward TwoMeters", new ChassisDriveDistance(2_m, 0.5_mps, 5_s, &m_drivetrain));
    m_autonomousChooser.AddOption("Led Autonomous",          new AutonomousLed(&m_leds));
    m_autonomousChooser.AddOption("Parallel Test",           new AutonomousParallel(&m_leds,             &m_drivetrain));
    m_autonomousChooser.AddOption("Complex Test",            new AutonomousComplex(&m_leds,              &m_drivetrain));

    // Send the autonomous mode chooser to the SmartDashboard
    frc::SmartDashboard::PutData("Autonomous Mode", &m_autonomousChooser);

    // Set the default commands for the subsystems
    m_drivetrain.SetDefaultCommand(ChassisDrive(
        [this] { return Forward(); },
        [this] { return Strafe();  },
        [this] { return Angle();   },
        &m_drivetrain));

    m_leds.SetDefaultCommand(SetLeds(LedMode::Off, &m_leds));
}
#pragma endregion

#pragma region ConfigureButtonBindings
/// @brief Method to bind the joystick controls to the robot commands.
void RobotContainer::ConfigureButtonBindings()
{
    // Bind the driver controller buttons to the robot commands
    frc2::JoystickButton fieldCentricOn(&m_driverController, Extreme3DContants::HandleLowerLeft);
    fieldCentricOn.OnTrue(ChassisSetFieldCentricity(true, &m_drivetrain).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::JoystickButton fieldCentricOff(&m_driverController, Extreme3DContants::HandleLowerRight);
    fieldCentricOff.OnTrue(ChassisSetFieldCentricity(false, &m_drivetrain).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    // Bind the operator controller buttons to the robot commands
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX).WhileTrue(new frc2::RunCommand([this] { m_drivetrain.SetX(); }, {&m_drivetrain}));

    frc2::JoystickButton setLedsOff(&m_operatorController, XBoxConstants::LeftStickButton);
    setLedsOff.OnTrue(SetLeds(LedMode::Off, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::JoystickButton setLedsStrobe(&m_operatorController, XBoxConstants::RightStickButton);
    setLedsStrobe.OnTrue(SetLeds(LedMode::Strobe, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));

    frc2::JoystickButton setLedsShootingAnimation{&m_operatorController, XBoxConstants::A};
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
    return -m_xspeedLimiter.Calculate(frc::ApplyDeadband(joystickForward, ControllerConstants::JoystickDeadZone)) * ChassisConstants::MaxSpeed;
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
    return -m_yspeedLimiter.Calculate(frc::ApplyDeadband(joystickStrafe, ControllerConstants::JoystickDeadZone)) * ChassisConstants::MaxSpeed;
}
#pragma endregion

#pragma region Angle
/// @brief Method to return the angle joystick value.
/// @return The angle joystick value.
units::radians_per_second_t RobotContainer::Angle()
{
    // Get the angle joystick setting
    double joystickAngle = GetDriverController()->GetRawAxis(ControllerConstants::JoystickAngleIndex);

    // Use exponential function to calculate the forward value for better slow speed control
    joystickAngle = GetExponentialValue(joystickAngle, ControllerConstants::ExponentAngle);

    // Return the rotation speed
    return -m_rotLimiter.Calculate(frc::ApplyDeadband(joystickAngle, ControllerConstants::JoystickRotateDeadZone)) * ChassisConstants::MaxAngularSpeed;
}
#pragma endregion

#pragma region GetExponentialValue
/// @brief Method to convert a joystick value from -1.0 to 1.0 to exponential mode.
/// @param joystickValue The raw joystick value.
/// @param exponent The exponential value.
/// @return The resultant exponential value.
double RobotContainer::GetExponentialValue(double joystickValue, double exponent)
{
    int    direction = 1;
    double output    = 0.0;

    // Direction is either 1 or -1, based on joystick value
    if (joystickValue < 0.0)
    {
        // Reverse the direction and make the joystick value positive
        direction      = -1;
        joystickValue *= -1.0;
    }

    // Plug joystick value into exponential function
    output = direction * pow(joystickValue, exponent);

    // Ensure the range of the output
    if (output < -1.0)  output = -1.0;
    if (output >  1.0)  output =  1.0;

    // Return the calculated value
    return output;
}

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

#pragma region GetTrajectoryCommand
/// @brief Method to return a pointer to the autonomous command.
/// @return Pointer to the autonomous command
frc2::Command *RobotContainer::GetTrajectoryCommand()
{
    // Set up config for trajectory
    frc::TrajectoryConfig config(AutoConstants::MaxSpeed, AutoConstants::MaxAcceleration);

    // Add kinematics to ensure max speed is actually obeyed
    config.SetKinematics(m_drivetrain.m_kinematics);

    // An example trajectory to follow.  All units in meters.
    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        // Start at the origin facing the +X direction
        frc::Pose2d{0_m, 0_m, 0_deg},
        // Pass through these two interior waypoints, making an 's' curve path
        {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
        // End 3 meters straight ahead of where we started, facing forward
        frc::Pose2d{3_m, 0_m, 0_deg},
        // Pass the config
        config);

    frc::ProfiledPIDController<units::radians> thetaController{
        AutoConstants::PThetaController, 0, 0,
        AutoConstants::ThetaControllerConstraints};

    thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                          units::radian_t{std::numbers::pi});

    frc2::SwerveControllerCommand<4> swerveControllerCommand(
        exampleTrajectory, [this]()
        { return m_drivetrain.GetPose(); },

        m_drivetrain.m_kinematics,

        frc::PIDController{AutoConstants::PXController, 0, 0},
        frc::PIDController{AutoConstants::PYController, 0, 0}, thetaController,

        [this](auto moduleStates)
        { m_drivetrain.SetModuleStates(moduleStates); },

        {&m_drivetrain});

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.ResetOdometry(exampleTrajectory.InitialPose());

    return new frc2::SequentialCommandGroup(std::move(swerveControllerCommand),
               frc2::InstantCommand([this]() { m_drivetrain.Drive(0_mps, 0_mps, 0_rad_per_s); }, {}));
}
#pragma endregion
