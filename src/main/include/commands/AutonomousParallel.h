// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/Leds.h"
#include "subsystems/Drivetrain.h"

#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

class AutonomousParallel : public frc2::CommandHelper<frc2::ParallelCommandGroup, AutonomousParallel>
{
    public:

        AutonomousParallel(Leds *m_leds, Drivetrain *m_drivetrain);
};
