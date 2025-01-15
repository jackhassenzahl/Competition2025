#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/Drivetrain.h"

class ChassisDrive : public frc2::CommandHelper<frc2::Command, ChassisDrive>
{
    public:

        explicit ChassisDrive(std::function<double()> forward, std::function<double()> strafe, 
                              std::function<double()> angle, std::function<double()> gyro, Drivetrain *drivetrain);

        void     Initialize() override;
        void     Execute()    override;

    private:

        std::function<double()> m_forward;
        std::function<double()> m_strafe;
        std::function<double()> m_angle;
        std::function<double()> m_gyro;

        Drivetrain *m_drivetrain;
};
