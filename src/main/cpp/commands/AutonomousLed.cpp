// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutonomousLed.h"
#include "commands/SetLeds.h"

AutonomousLed::AutonomousLed(Leds *leds)
{
  // Use addRequirements() here to declare subsystem dependencies.

  // Set the command name
  SetName("AutonomusLed");

  AddCommands(
    SetLeds(LedMode::Rainbow, leds, 5),
    SetLeds(LedMode::Strobe, leds, 5),
    SetLeds(LedMode::HvaColors, leds, 5)
  );
}

