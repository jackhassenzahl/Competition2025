#pragma once

#include "subsystems/Drivetrain.h"
#include "subsystems/Leds.h"
#include "subsystems/AprilTags.h"

#include "commands/AutonomousCommand.h"
#include "commands/ChassisDrive.h"
#include "commands/DriveDistance.h"
#include "commands/SetLeds.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>

#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>

class RobotContainer
{
    public:

        frc2::Command         *GetAutonomousCommand();
        static RobotContainer *GetInstance();

        // The robot's subsystems
        AprilTags  m_aprilTags;
        Drivetrain m_drivetrain;
        Leds       m_leds;

        frc::Joystick *getJoystickDriver();
        frc::Joystick *getJoystickOperator();

    private:

        RobotContainer();

        // Joysticks
        frc::Joystick m_joystickDriver{0};
        frc::Joystick m_joystickOperator{1};

        frc::SendableChooser<frc2::Command *> m_chooser;

        static RobotContainer *m_robotContainer;

       void ConfigureButtonBindings();
};
