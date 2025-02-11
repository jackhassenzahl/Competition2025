#include "commands/ArmSetAngle.h"

#pragma region ArmSetAngle
/// @brief The Constructor for the ArmSetAngle class.
/// @param setArmAngleTo The angle to set the arm.
/// @param arm The arm subsystem.
ArmSetAngle::ArmSetAngle(units::angle::degree_t setArmAngleTo, Arm *arm) : m_angle(setArmAngleTo), m_arm(arm)
{
    // Set the command name
    SetName("ArmSetAngle");

    // Declare subsystem dependencies
    AddRequirements(arm);
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void ArmSetAngle::Execute()
{
    // Set the arm angle
    m_arm->SetAngle(m_angle);
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool ArmSetAngle::IsFinished()
{
    // Execute only runs once
    return true;
}
#pragma endregion
