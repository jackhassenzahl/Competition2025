#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/Drivetrain.h"

#include "Constants.h"

class ChassisDriveTime : public frc2::CommandHelper<frc2::Command, ChassisDriveTime>
{
    public:

        explicit ChassisDriveTime(units::second_t time, double speed, Drivetrain *drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

    private:

        units::second_t m_time;             // The length of time that the chass will drive
        double          m_speed;            // The speed of the chassis
        Drivetrain     *m_drivetrain;       // The drivetrain subsystem

        bool            m_fieldCentricity;  // The field centricity flag
        units::second_t m_startTime;        // The start of the drive time
};
