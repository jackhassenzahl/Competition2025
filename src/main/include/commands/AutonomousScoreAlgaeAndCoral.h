#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Gripper.h"
#include "subsystems/AprilTags.h"

#include "commands/AutonomousOneCoral.h"
#include "commands/AprilTagGrabAlgae.h"
#include "commands/GripperActivate.h"
#include "commands/ChassisDrivePose.h"
#include "commands/GripperPose.h"

#include <constants/xBoxConstants.h>
#include <constants/Extreme3DConstants.h>
#include <constants/ControllerConstants.h>
#include <constants/ControlPanelConstants.h>
#include <constants/CanConstants.h>

#include <constantspose/ActivateConstants.h>
#include <constantspose/AlgaePoseConstants.h>
#include <constantspose/AprilTagToPoseConstants.h>
#include <constantspose/CoralPoseConstants.h>
#include <constantspose/AutonomousConstants.h>

// #include "Constants.h"
// #include "ConstantsPose.h"

class AutonomousScoreAlgaeAndCoral : public frc2::CommandHelper<frc2::SequentialCommandGroup, AutonomousScoreAlgaeAndCoral>
{
    public:

        explicit AutonomousScoreAlgaeAndCoral(Drivetrain *drivetrain, Gripper *gripper, AprilTags* aprilTags);
};
