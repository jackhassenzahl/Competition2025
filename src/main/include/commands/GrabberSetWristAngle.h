#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Grabber.h"

class GrabberSetWristAngle : public frc2::CommandHelper<frc2::Command, GrabberSetWristAngle>
{
    public:
        GrabberSetWristAngle(units::angle::degree_t setGrabberAngleTo, Grabber *grabber);

        void Execute()    override;
        bool IsFinished() override;
    
    private:
        units::angle::degree_t     m_angle;
        Grabber                   *m_grabber;
};
