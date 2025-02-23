#pragma once

#include <frc2/command/CommandHelper.h>

#include <frc2/command/SequentialCommandGroup.h>
#include "commands/ChassisDriveToAprilTag.h"
#include "commands/GripperPose.h"

#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"
#include "subsystems/AprilTags.h"
#include <functional>

class AutonomusScoreCoral : public frc2::CommandHelper<frc2::SequentialCommandGroup,  AutonomusScoreCoral>
{
    public:

        explicit AutonomusScoreCoral(GripperPoseEnum               gripperPose,
                                     const std::function<bool ()> &GetJoystickToggle,
                                     AprilTags                    *aprilTags,
                                     Gripper                      *gripper,
                                     Drivetrain                   *drivetrain);
};
