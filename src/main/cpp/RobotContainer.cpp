#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ParallelRaceGroup.h>

#include "Constants.h"
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
    // SmartDashboard Buttons
    frc::SmartDashboard::PutData("SetLeds: Off",               new SetLeds(LedMode::Off,               &m_leds));
    frc::SmartDashboard::PutData("SetLeds: SolidGreen",        new SetLeds(LedMode::SolidGreen,        &m_leds));
    frc::SmartDashboard::PutData("SetLeds: SolidRed",          new SetLeds(LedMode::SolidRed,          &m_leds));
    frc::SmartDashboard::PutData("SetLeds: HvaColors",         new SetLeds(LedMode::HvaColors,         &m_leds));
    frc::SmartDashboard::PutData("SetLeds: Strobe",            new SetLeds(LedMode::Strobe,            &m_leds));
    frc::SmartDashboard::PutData("SetLeds: ShootingAnimation", new SetLeds(LedMode::ShootingAnimation, &m_leds));
    frc::SmartDashboard::PutData("SetLeds: Rainbow",           new SetLeds(LedMode::Rainbow,           &m_leds));

    frc::SmartDashboard::PutData("ChassisDrive: Stop",         new ChassisDriveDistance(0, 0.0,               &m_drivetrain));
    frc::SmartDashboard::PutData("DriveDistance: OneFoot",     new ChassisDriveDistance(1, 0.5,               &m_drivetrain));
    frc::SmartDashboard::PutData("DriveDistance: TwoFeet",     new ChassisDriveDistance(2, 0.5,               &m_drivetrain));

    // Bind the joystick controls to the robot commands
    ConfigureButtonBindings();

    // Configure the autonomous command chooser
    m_autonomousChooser.SetDefaultOption("Do Nothing",     new AutonomousDoNothing());
    m_autonomousChooser.AddOption("Drive Forward OneFoot", new ChassisDriveDistance(1, 0.5, &m_drivetrain));
    m_autonomousChooser.AddOption("Drive Forward TwoFeet", new ChassisDriveDistance(2, 0.5, &m_drivetrain));
    m_autonomousChooser.AddOption("Led Autonomous",        new AutonomousLed(&m_leds));
    m_autonomousChooser.AddOption("Parallel Test",         new AutonomousParallel(&m_leds, &m_drivetrain));
    m_autonomousChooser.AddOption("Complex Test",          new AutonomousComplex(&m_leds, &m_drivetrain));

    // Send the autonomous mode chooser to the SmartDashboard
    frc::SmartDashboard::PutData("Autonomous Mode", &m_autonomousChooser);

    // Set the default commands for the subsystems
    m_drivetrain.SetDefaultCommand(ChassisDrive(
        [this] { return Forward(); },
        [this] { return Strife();  },
        [this] { return Angle();   }, 
        [this] { return Gyro();    },
        &m_drivetrain));

    m_leds.SetDefaultCommand(SetLeds(LedMode::Off, &m_leds));
}

/// @brief Method to bind the joystick controls to the robot commands.
void RobotContainer::ConfigureButtonBindings()
{
    frc2::JoystickButton m_setLEDsOff{&m_joystickDriver, 1};
    frc2::JoystickButton m_setLEDsRainbow{&m_joystickDriver, 2};

    m_setLEDsOff.OnTrue(SetLeds(0, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
    m_setLEDsRainbow.OnTrue(SetLeds(1, &m_leds).WithInterruptBehavior(frc2::Command::InterruptionBehavior::kCancelSelf));
}

/// @brief Method to return a pointer to the driver joystick.
/// @return Pointer to the driver joystick.
frc::Joystick *RobotContainer::GetJoystickDriver()
{
    // Return the pointer to the driver joystick
    return &m_joystickDriver;
}

/// @brief Method to return a pointer to the controller joystick.
/// @return Pointer to the controller joystick.
frc::Joystick *RobotContainer::GetJoystickOperator()
{
    // Return the pointer to the operator joystick
    return &m_joystickOperator;
}

/// @brief Method to return a pointer to the autonomous command.
/// @return Pointer to the autonomous command
frc2::Command *RobotContainer::GetAutonomousCommand()
{
    // The selected command will be run in autonomous
    return m_autonomousChooser.GetSelected();
}

/// @brief Method to return the forward joystick value.
/// @return The forward joystick value.
double RobotContainer::Forward()
{
    double joystickForward = -GetJoystickDriver()->GetRawAxis(JoystickConstants::kJoystickForwardIndex);

    return GetExponentialValue(joystickForward, JoystickConstants::kExponentForward);
}

/// @brief Method to return the strife joystick value.
/// @return The strife joystick value.
double RobotContainer::Strife()
{
    double joystickStrife = GetJoystickDriver()->GetRawAxis(JoystickConstants::kJoystickStrifeIndex);

    return GetExponentialValue(joystickStrife, JoystickConstants::kExponentStrife);
}

/// @brief Method to return the angle joystick value.
/// @return The angle joystick value.
double RobotContainer::Angle()
{
    double joystickAngle = GetJoystickDriver()->GetRawAxis(JoystickConstants::kJoystickAngleIndex);

    return GetExponentialValue(joystickAngle, JoystickConstants::kExponentAngle);
}

/// @brief 
/// @return 
double RobotContainer::Gyro()
{
    // Test using joystick
    double joystickGyro = GetJoystickDriver()->GetRawAxis(5);

    return GetExponentialValue(joystickGyro, JoystickConstants::kExponentAngle);
}

/// <summary>
/// Method to convert a joystick value from -1.0 to 1.0 to exponential mode.
/// </summary>
/// <param name="joystickValue">The raw joystick value.</param>
/// <returns>The resultant exponential value.</returns>
double RobotContainer::GetExponentialValue(double joystickValue, double exponent)
{
    double output = 0.0;

    // Ignore joystick input if it's too small
    if (abs(joystickValue) < JoystickConstants::kJoystickDeadZone)
        return 0.0;

    // Direction is either 1 or -1, based on joystick value
    int direction = abs(joystickValue) / joystickValue;

    // Plug joystick value into exponential function
    output = direction * pow(abs(joystickValue), exponent);
    
    // Ensure the range of the output
    if (output < -1.0)  output = -1.0;
    if (output >  1.0)  output =  1.0;

    // Return the calculated value
    return output;
}
