#include "commands/AprilTagScoreCoral.h"

AprilTagScoreCoral::AprilTagScoreCoral(GripperPoseEnum               gripperPose,
                                       const std::function<bool ()> &GetJoystickToggle,
                                       AprilTags                    *aprilTags,
                                       Gripper                      *gripper,
                                       Drivetrain                   *drivetrain)
{
    // Get the state of the joystick toggle
    if (GetJoystickToggle())
        AddCommands(ChassisDriveToAprilTag(AprilTagToPoseConstants::ChassisSpeed,
                                           AprilTagToPoseConstants::CoralReefDistanceOffsetX,
                                           AprilTagToPoseConstants::CoralReefDistanceOffsetY,
                                           AprilTagToPoseConstants::CoralReefAngleOffset,
                                           AprilTagToPoseConstants::TimeoutTime,
                                           aprilTags, drivetrain), GripperPose(gripperPose, gripper));
    else
        AddCommands(ChassisDriveToAprilTag(AprilTagToPoseConstants::ChassisSpeed,
                                          -AprilTagToPoseConstants::CoralReefDistanceOffsetX,
                                           AprilTagToPoseConstants::CoralReefDistanceOffsetY,
                                           AprilTagToPoseConstants::CoralReefAngleOffset,
                                           AprilTagToPoseConstants::TimeoutTime,
                                           aprilTags, drivetrain), GripperPose(gripperPose, gripper));
}
