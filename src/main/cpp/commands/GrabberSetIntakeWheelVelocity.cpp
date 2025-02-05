#include "commands/GrabberSetIntakeWheelVelocity.h"

GrabberSetIntakeWheelVelocity::GrabberSetIntakeWheelVelocity(double setGrabberVelocityTo, Grabber *grabber) :
                             m_velocity(setGrabberVelocityTo),  m_grabber(grabber)
{
    // Set the command name
    SetName("GrabberSetIntakeWheelVelocity");
    // asdf
    
    // Declare subsystem dependencies
    AddRequirements(grabber);
}

// Called repeatedly when this Command is scheduled to run
void GrabberSetIntakeWheelVelocity::Execute() 
{
    m_grabber->SetGrabberWheelsVelocity(m_velocity);
}

/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool GrabberSetIntakeWheelVelocity::IsFinished() {
    return true;
}
