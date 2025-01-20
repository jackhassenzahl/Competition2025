#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "RobotContainer.h"

#include "subsystems/Drivetrain.h"

#include "Constants.h"

class ChassisTurnAngle : public frc2::CommandHelper<frc2::Command, ChassisTurnAngle>
{
    public:

        explicit ChassisTurnAngle(units::angle::degrees angle, units::meters_per_second_t speed, units::time::second_t timeoutTime, Drivetrain *drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        void     End(bool interrupted) override;
        bool     IsFinished()          override;

    private:

        units::angle::degrees       m_angle;        // The angle that the chassis will turn
        units::meters_per_second_t  m_speed;        // The speed that the chassis will turn
        units::time::second_t       m_timeoutTime;  // The time that the chassis will turn
        Drivetrain                 *m_drivetrain;   // The drivetrain subsystem

        units::time::second_t       m_startTime;    // The start of the turn time  
};
