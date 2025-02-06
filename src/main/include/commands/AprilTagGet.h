#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/AprilTags.h"

#include <iostream>

class AprilTagGet : public frc2::CommandHelper<frc2::Command, AprilTagGet>
{
    public:

        AprilTagGet(double aprilTagId, AprilTags *aprilTags);

        void Execute()    override;
        bool IsFinished() override;

    private:

        int                  m_aprilTagId;
        AprilTags           *m_aprilTags;
        AprilTagInformation  m_aprilTagInformation;
};
