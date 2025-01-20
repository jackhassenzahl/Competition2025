#include "RobotContainer.h"

// Reference to the RobotContainer singleton class
RobotContainer *RobotContainer::m_robotContainer = NULL;  

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

/// @brief Method to configure the robot and SmartDashboard configuration.
RobotContainer::RobotContainer()
{
    frc::SmartDashboard::PutData("ChassisDrive: Stop",         new ChassisDriveDistance(0_m, 0_mps,  &m_drivetrain));
    frc::SmartDashboard::PutData("DriveDistance: OneMeter",    new ChassisDriveDistance(1_m, 0.5_mps,  &m_drivetrain));
    frc::SmartDashboard::PutData("DriveDistance: TwoMeters",   new ChassisDriveDistance(2_m, 0.5_mps,  &m_drivetrain));

    // Bind the joystick controls to the robot commands
    ConfigureButtonBindings();

    // Configure the autonomous command chooser
    m_autonomousChooser.SetDefaultOption("Do Nothing",       new AutonomousDoNothing());
    m_autonomousChooser.AddOption("Drive Forward OneMeter",  new ChassisDriveDistance(1_m, 0.5_mps, &m_drivetrain));
    m_autonomousChooser.AddOption("Drive Forward TwoMeters", new ChassisDriveDistance(2_m, 0.5_mps, &m_drivetrain));
    m_autonomousChooser.AddOption("Led Autonomous",          new AutonomousLed(&m_leds));
    m_autonomousChooser.AddOption("Parallel Test",           new AutonomousParallel(&m_leds, &m_drivetrain));
    m_autonomousChooser.AddOption("Complex Test",            new AutonomousComplex(&m_leds, &m_drivetrain));

    // Send the autonomous mode chooser to the SmartDashboard
    frc::SmartDashboard::PutData("Autonomous Mode", &m_autonomousChooser);

    // Set the default commands for the subsystems
    m_drivetrain.SetDefaultCommand(ChassisDrive(
        [this] { return Forward(); },
        [this] { return Strife();  },
        [this] { return Angle();   }, 
        &m_drivetrain));

    m_leds.SetDefaultCommand(SetLeds(LedMode::Off, &m_leds));
}

/// @brief Method to bind the joystick controls to the robot commands.
void RobotContainer::ConfigureButtonBindings()
{
    // Bind the driver controller buttons to the drivetrain commands
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

/// @brief Method to return a pointer to the driver joystick.
/// @return Pointer to the driver joystick.
frc::Joystick *RobotContainer::GetDriverController()
{
    // Return the pointer to the driver joystick
    return &m_driverController;
}

/// @brief Method to return a pointer to the controller joystick.
/// @return Pointer to the controller joystick.
frc::XboxController *RobotContainer::GetOperatorController()
{
    // Return the pointer to the operator joystick
    return &m_operatorController;
}

/// @brief Method to return a pointer to the autonomous command.
/// @return Pointer to the autonomous command
frc2::Command *RobotContainer::GetAutonomousCommand()
{
    // The selected command will be run in autonomous
    return m_autonomousChooser.GetSelected();
}

/// @brief Method to return the forward joystick value.
/// @return The forward joystick meters per second value.
units::meters_per_second_t RobotContainer::Forward()
{
    // Get the forward joystick setting
    double joystickForward = GetDriverController()->GetRawAxis(ControllerConstants::JoystickForwardIndex);

    // Get the x speed. We are inverting this because Xbox controllers return negative values when we push forward.
    joystickForward = GetExponentialValue(joystickForward, ControllerConstants::ExponentForward);

    // Return the x speed
    return -m_xspeedLimiter.Calculate(frc::ApplyDeadband(joystickForward, ControllerConstants::JoystickDeadZone)) * Drivetrain::kMaxSpeed;
}

/// @brief Method to return the strife joystick value.
/// @return The strife joystick meters per second value.
units::meters_per_second_t RobotContainer::Strife()
{
    // Get the strife joystick setting
    double joystickStrife = GetDriverController()->GetRawAxis(ControllerConstants::JoystickStrifeIndex);

    // Use expoendial function to calculate the forward value for better slow speed control
    joystickStrife = GetExponentialValue(joystickStrife, ControllerConstants::ExponentStrife);

    // Return the y speed
    return -m_yspeedLimiter.Calculate(frc::ApplyDeadband(joystickStrife, ControllerConstants::JoystickDeadZone)) * Drivetrain::kMaxSpeed;
}

/// @brief Method to return the angle joystick value.
/// @return The angle joystick value.
units::radians_per_second_t RobotContainer::Angle()
{
    // Get the angle joystick setting    
    double joystickAngle = GetDriverController()->GetRawAxis(ControllerConstants::JoystickAngleIndex);

    // Use expoendial function to calculate the forward value for better slow speed control
    joystickAngle = GetExponentialValue(joystickAngle, ControllerConstants::ExponentAngle);
    
    // Return the rotation speed
    return -m_rotLimiter.Calculate(frc::ApplyDeadband(joystickAngle, ControllerConstants::JoystickDeadZone)) * Drivetrain::kMaxAngularSpeed;
}

/// @brief Method to convert a joystick value from -1.0 to 1.0 to exponential mode.
/// @param joystickValue The raw joystick value.
/// @param exponent The exponential value.
/// @return The resultant exponential value.
double RobotContainer::GetExponentialValue(double joystickValue, double exponent)
{
    int    direction = 1;
    double output    = 0.0;

    // Ignore joystick input if it's too small
    if (joystickValue > -ControllerConstants::JoystickDeadZone && joystickValue < ControllerConstants::JoystickDeadZone)
        return 0.0;

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

/// @brief Method to set the timed robot period.
/// @param period The period to set.
void RobotContainer::SetPeriod(units::second_t period)
{
    // Set the period
    m_period = period;
}

/// @brief Method to get the timed robot period.
/// @return The timed robot period.
units::second_t RobotContainer::GetPeriod()
{
    // Return the timed robot period
    return m_period;
}
