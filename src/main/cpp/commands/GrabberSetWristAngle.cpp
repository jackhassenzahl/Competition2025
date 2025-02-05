#include "commands/GrabberSetWristAngle.h"

GrabberSetWristAngle::GrabberSetWristAngle(units::angle::degree_t setGrabberAngleTo, Grabber *grabber) :
                             m_angle(setGrabberAngleTo),  m_grabber(grabber)
{
    // Set the command name
    SetName("GrabberSetWristAngle");
    // asdf
    
    // Declare subsystem dependencies
    AddRequirements(grabber);
}

// Called repeatedly when this Command is scheduled to run
void GrabberSetWristAngle::Execute() 
{
    m_grabber->SetWristAngle(m_angle);
}

/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool GrabberSetWristAngle::IsFinished() {
    return true;
}
