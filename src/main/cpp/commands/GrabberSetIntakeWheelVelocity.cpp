#include "commands/GrabberSetIntakeWheelVelocity.h"

#pragma region GrabberSetIntakeWheelVelocity
/// @brief Constructor for the GrabberSetIntakeWheelVelocity class.
/// @param setGrabberVelocityTo The velocity to set the grabber to.
/// @param grabber The grabber subsystem.
GrabberSetIntakeWheelVelocity::GrabberSetIntakeWheelVelocity(double setGrabberVelocityTo, Grabber *grabber) :
                             m_velocity(setGrabberVelocityTo), m_grabber(grabber)
{
    // Set the command name
    SetName("GrabberSetIntakeWheelVelocity");

    // Declare subsystem dependencies
    AddRequirements(grabber);
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void GrabberSetIntakeWheelVelocity::Execute()
{
    m_grabber->SetGrabberWheelsVelocity(m_velocity);
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool GrabberSetIntakeWheelVelocity::IsFinished()
{
    // Execute only runs once
    return true;
}
#pragma endregion
