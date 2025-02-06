#include "commands/AprilTagGet.h"

#pragma region AprilTagGet
/// @brief Constructor for the AprilTagGet class.
/// @param aprilTagId The ID of the AprilTag to get.
/// @param aprilTags The AprilTags subsystem.
AprilTagGet::AprilTagGet(double aprilTagId, AprilTags *aprilTags) : m_aprilTagId(aprilTagId), m_aprilTags(aprilTags)
{
    // Set the command name
    SetName("AprilTagGet");

    // Declare subsystem dependencies
    AddRequirements(m_aprilTags);

    //m_aprilTagInformation = AprilTagInformation();
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void AprilTagGet::Execute()
{
    m_aprilTags->GetTag(m_aprilTagId, m_aprilTagInformation);
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool AprilTagGet::IsFinished()
{
    // Execute only runs once
    return true;
}
#pragma endregion
