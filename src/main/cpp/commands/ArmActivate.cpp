#include "commands/ArmActivate.h"

#pragma region ArmActivate
/// @brief Method to activate the arm to place a coral or a algae.
/// @param angleOffset The angle offset to move the arm.
/// @param grabberVoltage The voltage to apply to the grabber.
/// @param arm The arm subsystem.
ArmActivate::ArmActivate(units::angle::degree_t angleOffset, double grabberVoltage, Grabber *grabber, Arm *arm) :
                         m_angleOffset(angleOffset), m_grabberVoltage(grabberVoltage), m_grabber(grabber), m_arm(arm)
{
    // Set the command name
    SetName("ArmActivate");

    // Declare subsystem dependencies
    AddRequirements(arm);
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void ArmActivate::Execute()
{
    // Get the present arm angle and set the arm angle offset
    m_arm->SetAngle(m_arm->GetAngle() + m_angleOffset);

    // Set the grabber voltage
    m_grabber->SetGrabberWheelsVelocity(m_grabberVoltage);
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool ArmActivate::IsFinished()
{
    // Execute only runs once
    return true;
}
#pragma endregion
