#include "commands/GrabberSetWristAngle.h"

#pragma region GrabberSetWristAngle
/// @brief Constructor for the GrabberSetWristAngle class.
/// @param setGrabberAngleTo The angle to set the grabber to.
/// @param grabber The grabber subsystem.
GrabberSetWristAngle::GrabberSetWristAngle(units::angle::degree_t setGrabberAngleTo, Grabber *grabber) :
                                           m_angle(setGrabberAngleTo), m_grabber(grabber)
{
    // Set the command name
    SetName("GrabberSetWristAngle");

    // Declare subsystem dependencies
    AddRequirements(grabber);
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void GrabberSetWristAngle::Execute()
{
    // Set the grabber angle
    m_grabber->SetWristAngle(m_angle);
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool GrabberSetWristAngle::IsFinished()
{
    // Execute only runs once
    return true;
}
#pragma endregion
