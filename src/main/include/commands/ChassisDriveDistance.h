#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/Drivetrain.h"

class ChassisDriveDistance : public frc2::CommandHelper<frc2::Command, ChassisDriveDistance>
{
    public:

        explicit ChassisDriveDistance(double distance, double speed, Drivetrain *drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

    private:

        double      m_distance;         // The distance to drive
        double      m_speed;            // The speed of the chassis
        Drivetrain *m_drivetrain;       // The drivetrain subsystem;

        bool        m_fieldCentricity;  // The field centricity setting (true = field centric, false = robot centric)
};
