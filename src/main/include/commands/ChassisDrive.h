#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "RobotContainer.h"

#include "subsystems/Drivetrain.h"

class ChassisDrive : public frc2::CommandHelper<frc2::Command, ChassisDrive>
{
    public:

        explicit ChassisDrive(std::function<units::meters_per_second_t()> forward, std::function<units::meters_per_second_t()> strafe, 
                              std::function<units::radians_per_second_t()> angle, Drivetrain *drivetrain);

        void     Execute() override;

    private:

        std::function<units::meters_per_second_t()>  m_forward;
        std::function<units::meters_per_second_t()>  m_strafe;
        std::function<units::radians_per_second_t()> m_angle;
        Drivetrain                                  *m_drivetrain;
};
