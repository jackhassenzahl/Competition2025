// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ChassisTurnAngle.h"

ChassisTurnAngle::ChassisTurnAngle() {
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ChassisTurnAngle::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ChassisTurnAngle::Execute() {}

// Called once the command ends or is interrupted.
void ChassisTurnAngle::End(bool interrupted) {}

// Returns true when the command should end.
bool ChassisTurnAngle::IsFinished() {
  return false;
}
