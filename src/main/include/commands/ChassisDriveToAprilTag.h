#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "RobotContainer.h"

#include "subsystems/AprilTags.h"
#include "subsystems/Drivetrain.h"

class ChassisDriveToAprilTag : public frc2::CommandHelper<frc2::Command, ChassisDriveToAprilTag>
{
    public:

        explicit ChassisDriveToAprilTag(units::meters_per_second_t speed, units::time::second_t timeoutTime, AprilTags *aprilTags, Drivetrain *drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

    private:

        units::meters_per_second_t m_speed;            // The speed of the chassis
        units::time::second_t      m_timeoutTime;      // The command time-out time
        AprilTags                 *m_aprilTags;        // The AprilTag subsystem
        Drivetrain                *m_drivetrain;       // The drivetrain subsystem

        bool                       m_fieldCentricity;  // The field centricity setting (true = field centric, false = robot centric)
        units::second_t            m_startTime;        // The start of the drive time
};
