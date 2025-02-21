#include "commands/GripperPose.h"

#pragma region GripperPose
/// @brief Command to set the pose of the gripper.
/// @param gripperPose The pose to set the gripper.
/// @param gripper The gripper subsystem.
GripperPose::GripperPose(GripperPoseEnum gripperPose, Gripper *gripper) : m_gripperPose(gripperPose), m_gripper(gripper)
{
    // Set the command name
    SetName("GripperPose");

    // Declare subsystem dependencies
    AddRequirements(m_gripper);
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void GripperPose::Execute()
{
    // Set the gripper pose
    m_gripper->SetPose(m_gripperPose);
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool GripperPose::IsFinished()
{
    // Execute only runs once
    return true;
}
#pragma endregion
