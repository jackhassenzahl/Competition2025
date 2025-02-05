#include "commands/ClimbSetAngle.h"

ClimbSetAngle::ClimbSetAngle(units::angle::degree_t setClimbAngleTo, Climb *climb) :
                             m_angle(setClimbAngleTo),  m_climb(climb)
{
    // Set the command name
    SetName("ClimbSetAngle");
    // asdf
    
    // Declare subsystem dependencies
    AddRequirements(climb);
}

// Called repeatedly when this Command is scheduled to run
void ClimbSetAngle::Execute() 
{
    m_climb->SetAngle(m_angle);
}

/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool ClimbSetAngle::IsFinished() {
    return true;
}
