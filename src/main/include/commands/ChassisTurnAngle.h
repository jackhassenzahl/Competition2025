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

        units::meters_per_second_t  m_speed;
        units::time::second_t       m_timeoutTime;
        units::time::second_t       m_startTime;
        units::angle::degrees       m_angle;

        Drivetrain                 *m_drivetrain;        
};
