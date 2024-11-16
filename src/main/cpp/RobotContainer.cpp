#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/ParallelRaceGroup.h>

RobotContainer *RobotContainer::m_robotContainer = NULL;  // Reference to the RobotContainer singleton class

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

    frc::SmartDashboard::PutData("ChassisDrive: Stop",         new ChassisDrive(0, 0,                  &m_drivetrain));
    frc::SmartDashboard::PutData("DriveDistance: OneFoot",     new DriveDistance(1,                    &m_drivetrain));
    frc::SmartDashboard::PutData("DriveDistance: TwoFeet",     new DriveDistance(2,                    &m_drivetrain));

    // Bind the joystick controls to the robot commands
    ConfigureButtonBindings();

    // Configure the autonomous command chooser
    m_autonomousChooser.SetDefaultOption("Do Nothing",     new AutonomousDoNothing());
    m_autonomousChooser.AddOption("Drive Forward OneFoot", new DriveDistance(1, &m_drivetrain));
    m_autonomousChooser.AddOption("Drive Forward TwoFeet", new DriveDistance(2, &m_drivetrain));

    // Send the autonomous mode chooser to the SmartDashboard
    frc::SmartDashboard::PutData("Autonomous Mode", &m_autonomousChooser);

    // Set the default commands for the subsystems
    m_drivetrain.SetDefaultCommand(ChassisDrive(0, 0, &m_drivetrain));
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
frc::Joystick *RobotContainer::getJoystickDriver()
{
    // Return the pointer to the driver joystick
    return &m_joystickDriver;
}

/// @brief Method to return a pointer to the controller joystick.
/// @return Pointer to the controller joystick.
frc::Joystick *RobotContainer::getJoystickOperator()
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
