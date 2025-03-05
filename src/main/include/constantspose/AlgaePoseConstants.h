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

#pragma region AlgaePoseConstants
namespace AlgaePoseConstants
{
    constexpr auto GroundElevator          = 0.1_m;
    constexpr auto GroundArmAngle          = 100.0_deg;
    constexpr auto GroundWristAngle        = 0.0_deg;
    constexpr auto GroundGripperVoltage    = 0.0_V;

    constexpr auto OnCoralElevator         = 0.3_m;
    constexpr auto OnCoralArmAngle         = 90.0_deg;
    constexpr auto OnCoralWristAngle       = 0.0_deg;
    constexpr auto OnCoralGripperVoltage   = 0.0_V;

    constexpr auto LowElevator              = 0.8_m;
    constexpr auto LowArmAngle              = 90.0_deg;
    constexpr auto LowWristAngle            = 0.0_deg;
    constexpr auto LowGripperVoltage        = 0.0_V;

    constexpr auto HighElevator            = 1.22_m;
    constexpr auto HighArmAngle            = 90.0_deg;
    constexpr auto HighWristAngle          = 0.0_deg;
    constexpr auto HighGripperVoltage      = 0.0_V;

    constexpr auto ProcessorElevator       = 0.2_m;
    constexpr auto ProcessorArmAngle       = 100.0_deg;
    constexpr auto ProcessorWristAngle     = 0.0_deg;
    constexpr auto ProcessorGripperVoltage = 0.0_V;

    constexpr auto BargeElevator           = 1.5_m;
    constexpr auto BargeArmAngle           = 45.0_deg;
    constexpr auto BargeWristAngle         = 0.0_deg;
    constexpr auto BargeGripperVoltage     = 0.0_V;
}
#pragma endregion
