#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

#include "Constants.h"

class ChassisTurnAngle : public frc2::CommandHelper<frc2::Command, ChassisTurnAngle>
{
    public:

        explicit ChassisTurnAngle(units::angle::degrees angle, double speed, units::time::second_t timeoutTime, Drivetrain *drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        void     End(bool interrupted) override;
        bool     IsFinished()          override;

    private:

        units::angle::degrees m_angle;        // The angle to turn
        double                m_speed;        // The speed of the chassis
        units::time::second_t m_timeoutTime;  // The time to stop the turn
        Drivetrain           *m_drivetrain;   // The drivetrain subsystem      

        units::time::second_t m_startTime;    // The start of the turn time
};
