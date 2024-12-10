// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutonomousParallel.h"
#include "commands/SetLeds.h"
#include "commands/DriveTime.h"

AutonomousParallel::AutonomousParallel(Leds *leds, Drivetrain *drivetrain)
{
    // Set the command name
    SetName("AutonomousParallel");

    AddCommands(DriveTime(2_s, 0.5, drivetrain), 
                SetLeds(LedMode::Rainbow, 5_s, leds));
}
