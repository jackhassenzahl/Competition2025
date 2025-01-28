#pragma once

#include <frc/MathUtil.h>
#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc/Joystick.h>
#include <frc/filter/SlewRateLimiter.h>

// Subsystems
#include "subsystems/AprilTags.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Elevator.h"
#include "subsystems/Leds.h"

// Commands
#include "commands/AutonomousDoNothing.h"
#include "commands/AutonomousLed.h"
#include "commands/AutonomousParallel.h"
#include "commands/AutonomousComplex.h"
#include "commands/ChassisDrive.h"
#include "commands/ChassisDriveDistance.h"
#include "commands/ChassisDriveTime.h"
#include "commands/ChassisSetFieldCentricity.h"
#include "commands/ChassisSetSwerveWheelAnglesToZero.h"
#include "commands/SetLeds.h"

#include "Constants.h"

/// @brief Class to instantiate the robot subsystems and commands along with the operator controls
class RobotContainer
{
    public:

        // Method that returns a pointer to the singleton instance of the RobotContainer class
        static RobotContainer      *GetInstance();

        // Method to get a pointer to the selected autonomous command
        frc2::Command              *GetAutonomousCommand();

        // Method to set the swerve wheels to zero degrees based on the absolute encoder
        void                        SetSwerveWheelAnglesToZero();

        // Methods to get a reference to the robot joysticks
        frc::Joystick              *GetDriverController();
        frc::XboxController        *GetOperatorController();

        units::meters_per_second_t  Forward();
        units::meters_per_second_t  Strafe();
        units::radians_per_second_t Angle();

        void                        SetPeriod(units::second_t period);
        units::second_t             GetPeriod();

    private:

        // Private class constructor to configure the robot and SmartDashboard configuration
        RobotContainer();

        // Method to bind the joystick controls to the robot commands
        void   ConfigureButtonBindings();

        double GetExponentialValue(double joystickValue, double exponent);

        // Singleton reference to the class (returned by the GetInstance Method)
        static RobotContainer                *m_robotContainer;

        // Instantiate the robot subsystems
        AprilTags                             m_aprilTags;
        Drivetrain                            m_drivetrain;
        Elevator                              m_elevator;
        Leds                                  m_leds;

        units::second_t                       m_period;

        // Joysticks
        frc::Joystick                         m_driverController{ControllerConstants::DriverControllerUsbPort};
        frc::XboxController                   m_operatorController{ControllerConstants::JoystickOperatorUsbPort};

        // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
        frc::SlewRateLimiter<units::scalar>   m_xspeedLimiter{3 / 1_s};
        frc::SlewRateLimiter<units::scalar>   m_yspeedLimiter{3 / 1_s};
        frc::SlewRateLimiter<units::scalar>   m_rotLimiter   {3 / 1_s};

        // Create the command to set the swerve wheel angles to zero based on the absolute encoder
        ChassisSetSwerveWheelAnglesToZero    *m_swerveWheelAnglesToZero = new ChassisSetSwerveWheelAnglesToZero(&m_drivetrain);

        // Autonomous command chooser
        frc::SendableChooser<frc2::Command*> m_autonomousChooser;
};
