#include "commands/AprilTagGet.h"

AprilTagGet::AprilTagGet(double aprilTagId, AprilTags *aprilTags) :
                             m_aprilTagId(aprilTagId),  m_aprilTags(aprilTags)
{
    // Set the command name
    SetName("AprilTagGet");
    
    // Declare subsystem dependencies
    AddRequirements(m_aprilTags);

    //m_aprilTagInformation = AprilTagInformation();
}

// Called repeatedly when this Command is scheduled to run
void AprilTagGet::Execute() 
{
    m_aprilTags->GetTag(m_aprilTagId, m_aprilTagInformation);
}

/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool AprilTagGet::IsFinished() {
    return true;
}
