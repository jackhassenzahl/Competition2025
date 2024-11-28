#pragma once

#include "subsystems/Drivetrain.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

class DriveDistance : public frc2::CommandHelper<frc2::Command, DriveDistance>
{
    public:

        explicit DriveDistance(double distance, double speed, Drivetrain *m_drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

    private:

        bool        m_fieldCentricity;
        double      m_distance;
        double      m_speed;

        Drivetrain *m_drivetrain;
};
