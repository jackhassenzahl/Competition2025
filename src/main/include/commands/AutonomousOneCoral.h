#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"
#include "subsystems/AprilTags.h"

#include "commands/ChassisDrivePose.h"
#include "commands/AprilTagScoreCoral.h"
#include "commands/GripperActivate.h"

#include <constants/xBoxConstants.h>
#include <constants/Extreme3DConstants.h>
#include <constants/ControllerConstants.h>
#include <constants/ControlPanelConstants.h>
#include <constants/CanConstants.h>

// #include "Constants.h"
#include "ConstantsPose.h"

class AutonomousOneCoral : public frc2::CommandHelper<frc2::SequentialCommandGroup, AutonomousOneCoral>
{
    public:

        explicit AutonomousOneCoral(GripperPoseEnum gripperPoseEnum, std::function<ChassDrivePoseParameters()> getParameters, Drivetrain *drivetrain, Gripper *gripper, AprilTags* aprilTags);
};
