#pragma once

#include <units/time.h>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/Drivetrain.h"

class ChassisDriveTime : public frc2::CommandHelper<frc2::Command, ChassisDriveTime>
{
    public:

        explicit ChassisDriveTime(units::second_t time, double speed, Drivetrain *m_drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

    private:

        bool            m_fieldCentricity;
        double          m_speed;

        units::second_t m_time;           // The length of time that the chass will drive
        units::second_t m_startTime;      // The start of the drive time

        Drivetrain     *m_drivetrain;
};
