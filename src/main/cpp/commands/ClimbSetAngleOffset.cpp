#include "commands/ClimbSetAngleOffset.h"

ClimbSetAngleOffset::ClimbSetAngleOffset(units::angle::degree_t setClimbAngleTo, Climb *climb) : m_climbOffset(setClimbAngleTo), m_climb(climb)
{  
    // Set the command name
    SetName("ClimbSetAngleOffset");

    // Declare subsystem dependencies
    AddRequirements(climb);
}

// Called repeatedly when this Command is scheduled to run
void ClimbSetAngleOffset::Execute() 
{
    // Set the climb angle
    m_climb->SetAngle(m_climb->GetAngle() + m_climbOffset);
}

// Returns true when the command should end.
bool ClimbSetAngleOffset::IsFinished() {
  return true;
}
