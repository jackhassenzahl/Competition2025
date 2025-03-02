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

#include "Constants.h"
#include "ConstantsPose.h"

class AutonomousOneCoral : public frc2::CommandHelper<frc2::SequentialCommandGroup, AutonomousOneCoral>
{
    public:

        explicit AutonomousOneCoral(Drivetrain *drivetrain, Gripper *gripper, AprilTags* aprilTags);
};
