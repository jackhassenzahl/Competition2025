#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/Drivetrain.h"

class ChassisDriveDistance : public frc2::CommandHelper<frc2::Command, ChassisDriveDistance>
{
    public:

        explicit ChassisDriveDistance(units::meter_t distance, units::meters_per_second_t speed, Drivetrain *drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

    private:

        bool                       m_fieldCentricity;
        units::meter_t             m_distance;
        units::meters_per_second_t m_speed;

        Drivetrain *m_drivetrain;
};
