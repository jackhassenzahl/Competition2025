#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Grabber.h"

class GrabberSetIntakeWheelVelocity : public frc2::CommandHelper<frc2::Command, GrabberSetIntakeWheelVelocity>
{
    public:
        GrabberSetIntakeWheelVelocity(double setGrabberVelocityTo, Grabber *grabber);

        void Execute()    override;
        bool IsFinished() override;

    private:
        double   m_velocity;
        Grabber *m_grabber;
};
