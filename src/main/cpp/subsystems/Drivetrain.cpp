#include "subsystems/Drivetrain.h"
#include <frc/smartdashboard/SmartDashboard.h>

Drivetrain::Drivetrain()
{
    SetName("Drivetrain");
    SetSubsystem("Drivetrain");

    AddChild("Mecanum Drive", &m_mecanumDrive);
    m_mecanumDrive.SetSafetyEnabled(true);
    m_mecanumDrive.SetExpiration(0.1_s);
    m_mecanumDrive.SetMaxOutput(1.0);

    AddChild("Motor Controller 4", &m_motorController4);
    m_motorController4.SetInverted(false);

    AddChild("Motor Controller 3", &m_motorController3);
    m_motorController3.SetInverted(false);

    AddChild("Motor Controller 2", &m_motorController2);
    m_motorController2.SetInverted(false);

    AddChild("Motor Controller 1", &m_motorController1);
    m_motorController1.SetInverted(false);
}

void Drivetrain::Periodic()
{
    // Put code here to be run every loop
}

void Drivetrain::SimulationPeriodic()
{
    // This method will be called once per scheduler run when in simulation
}
