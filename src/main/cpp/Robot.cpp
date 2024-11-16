#include "Robot.h"

#include <hal/FRCUsageReporting.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

/// @brief Method called when the robot class is instantiated.
void Robot::RobotInit()
{
    EnableLiveWindowInTest(true);
    HAL_Report(HALUsageReporting::kResourceType_Framework, HALUsageReporting::kFramework_RobotBuilder);
}

/// @brief Method is called every robot packet, no matter the mode. 
void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
}

/// @brief Method is called once each time the robot enters Disabled mode.
void Robot::DisabledInit()
{

}

/// @brief Method is called periodically when the robot is disabled.
void Robot::DisabledPeriodic()
{

}

/// @brief Method is called when switching to teleoperated mode.
void Robot::TeleopInit()
{
    // This makes sure that the autonomous stops running when teleop starts running.
    if (m_autonomousCommand != nullptr)
    {
        // Cancel the autonomous command and set the pointer to null
        m_autonomousCommand->Cancel();
        m_autonomousCommand = nullptr;
    }
}

/// @brief Method is called periodically when the robot is in tleloperated mode.
void Robot::TeleopPeriodic()
{
  
}

/// @brief Method is called when switching to autonomous mode.
void Robot::AutonomousInit()
{
    // Get the selected autonomous command
    m_autonomousCommand = m_container->GetAutonomousCommand();

    // Determine if the chooser returned a pointer to a command
    if (m_autonomousCommand != nullptr)
    {
        // Schedule the autonomous command
        m_autonomousCommand->Schedule();
    }
}

/// @brief Method is called periodically when the robot is in autonomous mode.
void Robot::AutonomousPeriodic()
{

}

/// @brief Method is called when switching to test mode.
void Robot::TestInit()
{

}

// This function is called periodically during test mode.
void Robot::TestPeriodic()
{
  
}

/// @brief Method is called when starting in simulation mode.
void Robot::SimulationInit()
{
  
}

/// @brief Method is called periodically when in simulation mode.
void Robot::SimulationPeriodic()
{
  
}

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif
