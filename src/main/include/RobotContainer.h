#pragma once

#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Command.h>

// Subsystems
#include "subsystems/Drivetrain.h"
#include "subsystems/Leds.h"
#include "subsystems/AprilTags.h"

// Commands
#include "commands/AutonomousDoNothing.h"
#include "commands/AutonomousLed.h"
#include "commands/AutonomousParallel.h"
#include "commands/AutonomousComplex.h"
#include "commands/ChassisDrive.h"
#include "commands/ChassisDriveDistance.h"
#include "commands/ChassisDriveTime.h"
#include "commands/SetLeds.h"

/// @brief Class to instantiate the robot subsystems and commands along with the operator controls
class RobotContainer
{
    public:

        // Method that returns a pointer to the singleton instance of the RobotContainer class
        static RobotContainer      *GetInstance();

        // Method to get a pointer to the selected autonomous command
        frc2::Command              *GetAutonomousCommand();

        // Methods to get a reference to the robot joysticks
        frc::Joystick              *GetJoystickDriver();
        frc::Joystick              *GetJoystickOperator();

        units::meters_per_second_t  Forward();
        units::meters_per_second_t  Strife();
        units::radians_per_second_t Angle();

        // Instantiate the robot subsystems
        AprilTags  m_aprilTags;
        Drivetrain m_drivetrain;
        Leds       m_leds;

    private:

        // Private class constructor to configure the robot and SmartDashboard configuration
        RobotContainer();

        // Method to bind the joystick controls to the robot commands
        void   ConfigureButtonBindings();
        
        double GetExponentialValue(double joystickValue, double exponent);
        
        // Singleton reference to the class (returned by the GetInstance Method)
        static RobotContainer                *m_robotContainer;

        // Joysticks
        frc::Joystick                         m_joystickDriver{JoystickConstants::kJoystickDriverUsbPort};
        frc::Joystick                         m_joystickOperator{JoystickConstants::kJoystickOperatorUsbPort};

        // Autonomous command chooser
        frc::SendableChooser<frc2::Command *> m_autonomousChooser;
};
