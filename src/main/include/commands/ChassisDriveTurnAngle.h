#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "RobotContainer.h"

#include "subsystems/Drivetrain.h"

class ChassisDriveTurnAngle : public frc2::CommandHelper<frc2::Command, ChassisDriveTurnAngle>
{
    public:

        explicit ChassisDriveTurnAngle(units::angle::degree_t angle, units::meters_per_second_t speed, units::time::second_t timeoutTime, Drivetrain *drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

    private:

        units::angle::degree_t     m_angle;            // The angle to turn
        units::meters_per_second_t m_speed;            // The speed that the chassis will drive
        units::time::second_t      m_timeoutTime;      // The time to stop the turn
        Drivetrain                *m_drivetrain;       // The drivetrain subsystem

        bool                       m_fieldCentricity;  // The field centricity setting (true = field centric, false = robot centric)
        units::second_t            m_startTime;        // The start of the drive time
};
