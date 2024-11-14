#include "commands/SetLeds.h"

SetLeds::SetLeds(int Mode, Leds *m_leds) : m_Mode(Mode), m_leds(m_leds)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("SetLeds");
    AddRequirements({m_leds});
}

// Called just before this Command runs the first time
void SetLeds::Initialize()
{

}

// Called repeatedly when this Command is scheduled to run
void SetLeds::Execute()
{

}

// Make this return true when this Command no longer needs to run execute()
bool SetLeds::IsFinished()
{
    return false;
}

// Called once after isFinished returns true
void SetLeds::End(bool interrupted)
{

}

bool SetLeds::RunsWhenDisabled() const
{
    return true;
}
