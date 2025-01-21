#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/Drivetrain.h"

class ChassisDrive : public frc2::CommandHelper<frc2::Command, ChassisDrive>
{
    public:

        explicit ChassisDrive(std::function<double()> forward, std::function<double()> strafe, 
                              std::function<double()> angle, Drivetrain *drivetrain);

        void     Execute() override;

        void     SetFieldCentricity(bool fieldCentric);
        
    private:

        std::function<double()> m_forward;     // The forward speed
        std::function<double()> m_strafe;      // The strafe speed
        std::function<double()> m_angle;       // The angle speed
        Drivetrain             *m_drivetrain;  // The drivetrain subsystem;
};
