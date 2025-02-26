#include "commands/AutonomousOneCoral.h"

using namespace AutonomousConstants;

#pragma region AutonomousOneCoral (constructor)
/// @brief Command to place one coral in autonomous mode.
AutonomousOneCoral::AutonomousOneCoral(Drivetrain *drivetrain, Gripper *gripper, AprilTags *aprilTags)
{
    AddCommands(
        ChassisDrivePose(OneCoralSpeed, OneCoralXDistance, OneCoralYDistance, OneCoralAngleChange, OneCoralTimeOut, drivetrain),
        AprilTagScoreCoral(GripperPoseEnum::CoralL4, []() {return true;}, aprilTags, gripper, drivetrain),
        GripperActivate(gripper)
    );
}
#pragma endregion