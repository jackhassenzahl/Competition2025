#include "commands/ElevatorSetHeight.h"

#pragma region ElevatorSetHeight
/// @brief Constructor for the ElevatorSetHeight command.
/// @param height The height to set the elevator.
/// @param elevator The elevator subsystem.
ElevatorSetHeight::ElevatorSetHeight(units::length::meter_t height, Elevator *elevator) : m_height(height), m_elevator(elevator)
{
    // Set the command name
    SetName("Elevator");

    // Declare subsystem dependencies
    AddRequirements(elevator);
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void ElevatorSetHeight::Execute()
{
    // Set the elevator height setpoint
    m_elevator->SetHeight(m_height);
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool ElevatorSetHeight::IsFinished()
{
    // Execute only runs once
    return true;
}
#pragma endregion

