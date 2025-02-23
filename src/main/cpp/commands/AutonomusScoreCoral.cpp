#include "commands/AutonomusScoreCoral.h"
#include "commands/ChassisDriveToAprilTag.h"

AutonomusScoreCoral::AutonomusScoreCoral(GripperPoseEnum gripperPose,
                                         const std::function<bool ()>& GetJoystickToggle,
                                         AprilTags*      aprilTags,
                                         Gripper*        gripper,
                                         Drivetrain*     drivetrain
                                         ) 
{
  if (GetJoystickToggle())
  {
    AddCommands(
        ChassisDriveToAprilTag(1.0_mps, -1.0_m, 0.0_m, 0.0_deg, 10.0_s, aprilTags, drivetrain),
        GripperPose(gripperPose, gripper)
        );
  } else
  {
    
    AddCommands(
        ChassisDriveToAprilTag(1.0_mps, 1.0_m, 0.0_m, 0.0_deg, 10.0_s, aprilTags, drivetrain),
        GripperPose(gripperPose, gripper)
        );
  }
}