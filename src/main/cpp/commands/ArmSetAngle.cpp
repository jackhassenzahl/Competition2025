#include "commands/ArmSetAngle.h"

ArmSetAngle::ArmSetAngle(units::angle::degree_t setArmAngleTo, Arm *arm) :
                             m_angle(setArmAngleTo),  m_arm(arm)
{
    // Set the command name
    SetName("ArmSetAngle");
    // asdf
    
    // Declare subsystem dependencies
    AddRequirements(arm);
}

// Called repeatedly when this Command is scheduled to run
void ArmSetAngle::Execute() 
{
    m_arm->SetAngle(m_angle);
}

/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool ArmSetAngle::IsFinished() {
    return true;
}
