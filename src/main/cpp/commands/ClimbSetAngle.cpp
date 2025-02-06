#include "commands/ClimbSetAngle.h"

#pragma region ClimbSetAngle
/// @brief Constructor for the ClimbSetAngle class.
/// @param setClimbAngleTo The angle to set the climb to.
/// @param climb The climb subsystem.
ClimbSetAngle::ClimbSetAngle(units::angle::degree_t setClimbAngleTo, Climb *climb) : m_angle(setClimbAngleTo), m_climb(climb)
{
    // Set the command name
    SetName("ClimbSetAngle");

    // Declare subsystem dependencies
    AddRequirements(climb);
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void ClimbSetAngle::Execute()
{
    // Set the climb angle
    m_climb->SetAngle(m_angle);
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool ClimbSetAngle::IsFinished()
{
    // Execute only runs once
    return true;
}
#pragma endregion
