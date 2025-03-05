#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Gripper.h"

#include <constants/xBoxConstants.h>
#include <constants/Extreme3DConstants.h>
#include <constants/ControllerConstants.h>
#include <constants/ControlPanelConstants.h>
#include <constants/CanConstants.h>

// #include "Constants.h"

class GripperPose : public frc2::CommandHelper<frc2::Command, GripperPose>
{
    public:

        explicit GripperPose(GripperPoseEnum gripperPose, Gripper *gripper);

        void     Execute()             override;
        bool     IsFinished()          override;

    private:

        GripperPoseEnum m_gripperPose;  // The gripper pose
        Gripper        *m_gripper;      // The Gripper subsystem
};
