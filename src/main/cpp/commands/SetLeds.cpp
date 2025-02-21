#include "commands/SetLeds.h"

#pragma region SetLeds (constructor) - Mode and Leds
/// @brief Coinstructor for the SetLeds command.
/// @param Mode The LED mode.
/// @param m_leds The LED subsystem.
SetLeds::SetLeds(int Mode, Leds *leds) : m_mode(Mode), m_leds(leds)
{
    // Set the command name
    SetName("SetLeds");

    // Declare subsystem dependencies
    AddRequirements({m_leds});
}
#pragma endregion

#pragma region SetLeds (constructor) - Mode, Time and Leds
/// @brief Command to set the LED mode.
/// @param Mode The LED mode.
/// @param m_leds The LED subsystem.
SetLeds::SetLeds(int Mode, units::second_t time, Leds *leds) : m_mode(Mode), m_time(time), m_leds(leds)
{
    // Set the command name
    SetName("SetLeds");

    // Declare subsystem dependencies
    AddRequirements({m_leds});

    // Remember the LED mode
    m_mode = Mode;

    // Indicate that the LED sequence has a time-out
    m_timed = true;
}
#pragma endregion

#pragma region Initialize
/// @brief Called just before this Command runs.
void SetLeds::Initialize()
{
    // Set the LED mode
    m_leds->SetMode((LedMode) m_mode);

    // Get the LED sequence start time
    m_startTime = frc::GetTime();
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool SetLeds::IsFinished()
{
    // Determine if a timed LED sequence
    if (m_timed == false)
        return false;

    // Determine if the LED sequence is complete
    if (frc::GetTime() - m_startTime > m_time)
        return true;

    return false;
}
#pragma endregion

#pragma region RunsWhenDisabled
/// @brief Indicates if the command runs when the robot is disabled.
/// @return True is the command should run when the robot is disabled.
bool SetLeds::RunsWhenDisabled() const
{
    // Indicate that the command should run even when the robot is disabled
    return true;
}
#pragma endregion
