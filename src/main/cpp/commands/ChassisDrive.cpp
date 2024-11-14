#include "commands/ChassisDrive.h"

ChassisDrive::ChassisDrive(double Left, double Right, Drivetrain *m_drivetrain) : m_Left(Left), m_Right(Right), m_drivetrain(m_drivetrain)
{

    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("ChassisDrive");
    AddRequirements({m_drivetrain});
}

// Called just before this Command runs the first time
void ChassisDrive::Initialize()
{

}

// Called repeatedly when this Command is scheduled to run
void ChassisDrive::Execute()
{

}

// Make this return true when this Command no longer needs to run execute()
bool ChassisDrive::IsFinished()
{
    return false;
}

// Called once after isFinished returns true
void ChassisDrive::End(bool interrupted)
{

}

bool ChassisDrive::RunsWhenDisabled() const
{
    return false;
}
