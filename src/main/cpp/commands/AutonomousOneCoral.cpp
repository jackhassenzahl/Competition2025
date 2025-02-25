#include "commands/AutonomousOneCoral.h"

#pragma region AutonomousOneCoral (constructor)
/// @brief Command to place one coral in autonomous mode.
AutonomousOneCoral::AutonomousOneCoral(Drivetrain* drivetrain,
                                       Gripper*    gripper,
                                       AprilTags*  aprilTags)
{
    AddCommands(
        ChassisDrivePose(1_mps, 1_m, 0_m, 0_deg, 10_s, drivetrain),
        AutonomusScoreCoral(GripperPoseEnum::CoralL4, []() {return true;}, aprilTags, gripper, drivetrain),
        GripperActivate(gripper)
    );
}
#pragma endregion

// #pragma region Initialize
// /// @brief Called just before this Command runs.
// void AutonomousOneCoral::Initialize()
// {

// }
// #pragma endregion

// #pragma region Execute
// // Called repeatedly when this Command is scheduled to run.
// void AutonomousOneCoral::Execute()
// {

// }
// #pragma endregion

// #pragma region End
// // Called once the command ends or is interrupted.
// void AutonomousOneCoral::End(bool interrupted)
// {

// }
// #pragma endregion

// #pragma region IsFinished
// /// @brief Indicates if the command has completed.
// /// @return True is the command has completed.
// // Returns true when the command should end.
// bool AutonomousOneCoral::IsFinished()
// {
//   return false;
// }
// #pragma endregion
