#include "subsystems/Leds.h"
#include <frc/smartdashboard/SmartDashboard.h>

Leds::Leds()
{
    SetName("Leds");
    SetSubsystem("Leds");
}

void Leds::Periodic()
{
    // Put code here to be run every loop
}

void Leds::SimulationPeriodic()
{
    // This method will be called once per scheduler run when in simulation
}
