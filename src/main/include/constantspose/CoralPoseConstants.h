#pragma once

#pragma region Includes
#include <iostream>
#include <numbers>
#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#pragma endregion

#pragma region CoralPoseConstants
namespace CoralPoseConstants
{
    constexpr auto GroundElevator         = 0.103289_m;
    constexpr auto GroundArmAngle         = 138.765_deg;
    constexpr auto GroundWristAngle       = 180.0_deg;
    constexpr auto GroundGripperVoltage   = 0.0_V;

    constexpr auto StationElevator        = 0.5_m;
    constexpr auto StationArmAngle        = 45.0_deg;
    constexpr auto StationWristAngle      = 0.0_deg;
    constexpr auto StationGripperVoltage  = 0.0_V;

    constexpr auto L1Elevator             = 0.5_m;
    constexpr auto L1ArmAngle             = 80.0_deg;
    constexpr auto L1WristAngle           = 180.0_deg;
    constexpr auto L1GripperVoltage       = 0.0_V;

    constexpr auto L2Elevator             = 0.8_m;
    constexpr auto L2ArmAngle             = 45.0_deg;
    constexpr auto L2WristAngle           = 90.0_deg;
    constexpr auto L2GripperVoltage       = 0.0_V;

    constexpr auto L3Elevator             = 1.22_m;
    constexpr auto L3ArmAngle             = 45.0_deg;
    constexpr auto L3WristAngle           = 90.0_deg;
    constexpr auto L3GripperVoltage       = 1.0_V;

    constexpr auto L4Elevator             = 1.5_m;
    constexpr auto L4ArmAngle             = 45.0_deg;
    constexpr auto L4WristAngle           = 90.0_deg;
    constexpr auto L4GripperVoltage       = -1.0_V;
}
#pragma endregion
