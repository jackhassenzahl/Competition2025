#pragma once

#include "subsystems/Drivetrain.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

class ChassisDrive : public frc2::CommandHelper<frc2::Command, ChassisDrive>
{
    public:

        explicit ChassisDrive(std::function<double()> Left, std::function<double()> Right, std::function<double()> gyro, Drivetrain *m_drivetrain);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

        bool     RunsWhenDisabled()    const override;

    private:

        std::function<double()> m_left;
        std::function<double()> m_right;
        std::function<double()> m_gyro;

        Drivetrain *m_drivetrain;
};
