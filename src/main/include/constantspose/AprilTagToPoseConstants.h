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

#pragma region AprilTagToPoseConstants
namespace AprilTagToPoseConstants
{
    constexpr auto ChassisSpeed                  = 1.0_mps;
    constexpr auto TimeoutTime                   = 10_s;

    constexpr auto CoralStationDistanceOffsetX   = 0.0_m;
    constexpr auto CoralStationDistanceOffsetY   = 0.0_m;
    constexpr auto CoralStationAngleOffset       = 0.0_deg;

    constexpr auto CoralReefDistanceOffsetX      = 0.5_m;
    constexpr auto CoralReefDistanceOffsetY      = 0.5_m;
    constexpr auto CoralReefAngleOffset          = 0.0_deg;

    constexpr auto AlgaeReefDistanceOffsetX      = 0.0_m;
    constexpr auto AlgaeReefDistanceOffsetY      = 0.0_m;
    constexpr auto AlgaeReefAngleOffset          = 0.0_deg;

    constexpr auto AlgaeProcessorDistanceOffsetX = 0.0_m;
    constexpr auto AlgaeProcessorDistanceOffsetY = 0.0_m;
    constexpr auto AlgaeProcessorAngleOffset     = 0.0_deg;

    constexpr auto AlgaelBargeDistanceOffsetX    = 0.0_m;
    constexpr auto AlgaelBargeDistanceOffsetY    = 0.0_m;
    constexpr auto AlgaelBargeAngleOffset        = 0.0_deg;
}
#pragma endregion
