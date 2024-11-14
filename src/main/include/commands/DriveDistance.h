#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/Drivetrain.h"

class DriveDistance : public frc2::CommandHelper<frc2::Command, DriveDistance>
{
public:

    explicit DriveDistance(double Distance, Drivetrain *m_drivetrain);

    void     Initialize()          override;
    void     Execute()             override;
    bool     IsFinished()          override;
    void     End(bool interrupted) override;

    bool     RunsWhenDisabled()    const override;

private:

    double m_Distance;

    Drivetrain *m_drivetrain;
};
