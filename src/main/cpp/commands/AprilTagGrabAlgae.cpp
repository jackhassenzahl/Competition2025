#include "commands/AprilTagGrabAlgae.h"
#include "commands/ChassisDriveToAprilTag.h"

AprilTagGrabAlgae::AprilTagGrabAlgae(GripperPoseEnum               gripperPose,
                                       AprilTags                    *aprilTags,
                                       Gripper                      *gripper,
                                       Drivetrain                   *drivetrain)
{
    AddCommands(ChassisDriveToAprilTag(1.0_mps, 0.0_m, 0.0_m, 0.0_deg, 10.0_s, aprilTags, drivetrain),
                GripperPose(gripperPose, gripper),
                GripperActivate(gripper));
}
