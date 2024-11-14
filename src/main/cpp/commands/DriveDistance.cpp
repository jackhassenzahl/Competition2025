#include "commands/DriveDistance.h"

DriveDistance::DriveDistance(double Distance, Drivetrain *m_drivetrain) : m_Distance(Distance), m_drivetrain(m_drivetrain)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("DriveDistance");
    AddRequirements({m_drivetrain});
}

// Called just before this Command runs the first time
void DriveDistance::Initialize()
{

}

// Called repeatedly when this Command is scheduled to run
void DriveDistance::Execute()
{

}

// Make this return true when this Command no longer needs to run execute()
bool DriveDistance::IsFinished()
{
    return false;
}

// Called once after isFinished returns true
void DriveDistance::End(bool interrupted)
{
}

bool DriveDistance::RunsWhenDisabled() const
{
    return false;
}
