#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/Drivetrain.h"

class ChassisDrive : public frc2::CommandHelper<frc2::Command, ChassisDrive>
{
public:

    explicit ChassisDrive(double Left, double Right, Drivetrain *m_drivetrain);

    void     Initialize()          override;
    void     Execute()             override;
    bool     IsFinished()          override;
    void     End(bool interrupted) override;

    bool     RunsWhenDisabled()    const override;

private:

    double m_Left;
    double m_Right;

    Drivetrain *m_drivetrain;
};
