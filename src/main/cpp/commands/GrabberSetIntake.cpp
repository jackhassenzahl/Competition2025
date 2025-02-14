#include "commands/GrabberSetIntake.h"

#pragma region GrabberSetIntake
/// @brief Constructor for the GrabberSetIntake class.
/// @param angle The angle to set the arm.
/// @param velocity The velocity to set the grabber wheels.
/// @param grabber The grabber subsystem.
GrabberSetIntake::GrabberSetIntake(units::angle::degree_t angle, double velocity, Grabber *grabber) :
                                   m_angle(angle), m_velocity(velocity), m_grabber(grabber)
{
    // Set the command name
    SetName("GrabberSetIntake");

    // Declare subsystem dependencies
    AddRequirements(grabber);
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void GrabberSetIntake::Execute()
{
    // Set the grabber angle
    m_grabber->SetWristAngle(m_angle);

    // Set the grabber wheel velocity
    m_grabber->SetGrabberWheelsVelocity(m_velocity);
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool GrabberSetIntake::IsFinished()
{
    // Execute only runs once
    return true;
}
#pragma endregion
