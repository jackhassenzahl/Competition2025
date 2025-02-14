#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Grabber.h"

class GrabberSetIntake : public frc2::CommandHelper<frc2::Command, GrabberSetIntake>
{
    public:

        GrabberSetIntake(units::angle::degree_t angle, double velocity, Grabber *grabber);

        void Execute()    override;
        bool IsFinished() override;

    private:

        units::angle::degree_t m_angle;
        double                 m_velocity;
        Grabber               *m_grabber;
};
