#include "commands/SetLeds.h"

#include <frc/smartdashboard/SmartDashboard.h>

/// @brief Command to set the LED mode.
/// @param Mode The LED mode.
/// @param m_leds The LED subsystem.
SetLeds::SetLeds(int Mode, Leds *m_leds) : m_mode(Mode), m_leds(m_leds)
{
    // Set the command name
    SetName("SetLeds");

    // Declare subsystem dependencies
    AddRequirements({m_leds});

    // Remember the LED mode
    m_mode = Mode;

    m_time = -1;
}

/// @brief Command to set the LED mode.
/// @param Mode The LED mode.
/// @param m_leds The LED subsystem.
SetLeds::SetLeds(int Mode, int time, Leds *m_leds) : m_mode(Mode), m_time(time), m_leds(m_leds)
{
    // Set the command name
    SetName("SetLeds");

    // Declare subsystem dependencies
    AddRequirements({m_leds});

    // Remember the LED mode
    m_mode = Mode;

    // The length of time that the led's will be run in seconds
    m_time = time * LedConstants::kMillisecondsToSeconds;
}

/// @brief Called just before this Command runs the first time.
void SetLeds::Initialize()
{
    // Set the LED mode
    m_leds->SetMode((LedMode)m_mode);
}

/// @brief Indicates if the command has completed. Make this return true when this Command no longer needs to run execute().
/// @return True is the command has completed.
bool SetLeds::IsFinished()
{
    frc::SmartDashboard::PutNumber("Led Timer", m_time);

    // If m_time is NULL, then it should go on forever
    // So if m_time is NOT Null AND is less than 0, then the timer is over and it is finished
    if (m_time == -1)// && m_time-- <= 0)
    {
        // frc::SmartDashboard::PutBoolean("Finished", true);
        return false;
    }
    m_time--;
    if (m_time <= 0)
    {
        frc::SmartDashboard::PutBoolean("Finished", true);
        return true;
    }

    frc::SmartDashboard::PutBoolean("Finished", false);
    return false;
}

/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void SetLeds::End(bool interrupted)
{
    
}

/// @brief Indicates if the command runs when the robot is disabled.
/// @return True is the command should run when the robot is disabled.
bool SetLeds::RunsWhenDisabled() const
{
    return true;
}
