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

#pragma region ActivateConstants
namespace ActivateConstants
{
    // Coral Ground:
    constexpr auto CoralGroundElevatorOffset    = 0.0_m;
    constexpr auto CoralGroundWait1             = 0.0_s;
    constexpr auto CoralGroundArmOffset         = 0.0_deg;
    constexpr auto CoralGroundWait2             = 0.0_s;
    constexpr auto CoralGroundGripperVoltage    = 0.0_V;
    constexpr auto CoralGroundWait3             = 0.0_s;
    constexpr auto CoralGroundElevatorFinish    = 0.0_m;
    constexpr auto CoralGroundArmFinish         = 0.0_deg;

    // Coral Station:
    constexpr auto CoralStationElevatorOffset   = 0.0_m;
    constexpr auto CoralStationWait1            = 0.0_s;
    constexpr auto CoralStationArmOffset        = 0.0_deg;
    constexpr auto CoralStationWait2            = 0.0_s;
    constexpr auto CoralStationGripperVoltage   = 0.0_V;
    constexpr auto CoralStationWait3            = 0.0_s;
    constexpr auto CoralStationElevatorFinish   = 0.0_m;
    constexpr auto CoralStationArmFinish        = 0.0_deg;

    // Coral123:
    constexpr auto Coral123ElevatorOffset       =  0.2_m;
    constexpr auto Coral123Wait1                =  1.0_s;
    constexpr auto Coral123ArmOffset            =  10.0_deg;
    constexpr auto Coral123Wait2                =  0.1_s;
    constexpr auto Coral123GripperVoltage       = -3.0_V;
    constexpr auto Coral123Wait3                =  1.0_s;
    constexpr auto Coral123ElevatorFinish       =  0.2_m;
    constexpr auto Coral123ArmFinish            = 10.0_deg;

    // Coral4:
    constexpr auto Coral4ElevatorOffset         = 0.0_m;
    constexpr auto Coral4Wait1                  = 0.0_s;
    constexpr auto Coral4ArmOffset              = 0.0_deg;
    constexpr auto Coral4Wait2                  = 0.0_s;
    constexpr auto Coral4GripperVoltage         = 0.0_V;
    constexpr auto Coral4Wait3                  = 0.0_s;
    constexpr auto Coral4ElevatorFinish         = 0.0_m;
    constexpr auto Coral4ArmFinish              = 0.0_deg;

    // Algae Ground:
    constexpr auto AlgaeGroundElevatorOffset    = 0.0_m;
    constexpr auto AlgaeGroundWait1             = 0.0_s;
    constexpr auto AlgaeGroundArmOffset         = 0.0_deg;
    constexpr auto AlgaeGroundWait2             = 0.0_s;
    constexpr auto AlgaeGroundGripperVoltage    = 0.0_V;
    constexpr auto AlgaeGroundWait3             = 0.0_s;
    constexpr auto AlgaeGroundElevatorFinish    = 0.0_m;
    constexpr auto AlgaeGroundArmFinish         = 0.0_deg;

    // Algae On Coral:
    constexpr auto AlgaeOnCoralElevatorOffset   = 0.0_m;
    constexpr auto AlgaeOnCoralWait1            = 0.0_s;
    constexpr auto AlgaeOnCoralArmOffset        = 0.0_deg;
    constexpr auto AlgaeOnCoralWait2            = 0.0_s;
    constexpr auto AlgaeOnCoralGripperVoltage   = 0.0_V;
    constexpr auto AlgaeOnCoralWait3            = 0.0_s;
    constexpr auto AlgaeOnCoralElevatorFinish   = 0.0_m;
    constexpr auto AlgaeOnCoralArmFinish        = 0.0_deg;

    // Algae Lo:
    constexpr auto AlgaeLoElevatorOffset        = 0.0_m;
    constexpr auto AlgaeLoWait1                 = 0.0_s;
    constexpr auto AlgaeLoArmOffset             = 0.0_deg;
    constexpr auto AlgaeLoWait2                 = 0.0_s;
    constexpr auto AlgaeLoGripperVoltage        = 0.0_V;
    constexpr auto AlgaeLoWait3                 = 0.0_s;
    constexpr auto AlgaeLoElevatorFinish        = 0.0_m;
    constexpr auto AlgaeLoArmFinish             = 0.0_deg;

    // Algae High:
    constexpr auto AlgaeHighElevatorOffset      = 0.0_m;
    constexpr auto AlgaeHighWait1               = 0.0_s;
    constexpr auto AlgaeHighArmOffset           = 0.0_deg;
    constexpr auto AlgaeHighWait2               = 0.0_s;
    constexpr auto AlgaeHighGripperVoltage      = 0.0_V;
    constexpr auto AlgaeHighWait3               = 0.0_s;
    constexpr auto AlgaeHighElevatorFinish      = 0.0_m;
    constexpr auto AlgaeHighArmFinish           = 0.0_deg;

    // Algae Ground:
    constexpr auto AlgaeProcessorElevatorOffset = 0.0_m;
    constexpr auto AlgaeProcessorWait1          = 0.0_s;
    constexpr auto AlgaeProcessorArmOffset      = 0.0_deg;
    constexpr auto AlgaeProcessorWait2          = 0.0_s;
    constexpr auto AlgaeProcessorGripperVoltage = 0.0_V;
    constexpr auto AlgaeProcessorWait3          = 0.0_s;
    constexpr auto AlgaeProcessorElevatorFinish = 0.0_m;
    constexpr auto AlgaeProcessorArmFinish      = 0.0_deg;

    // Algae Ground:
    constexpr auto AlgaeBargeElevatorOffset     = 0.0_m;
    constexpr auto AlgaeBargeWait1              = 0.0_s;
    constexpr auto AlgaeBargeArmOffset          = 0.0_deg;
    constexpr auto AlgaeBargeWait2              = 0.0_s;
    constexpr auto AlgaeBargeGripperVoltage     = 0.0_V;
    constexpr auto AlgaeBargeWait3              = 0.0_s;
    constexpr auto AlgaeBargeElevatorFinish     = 0.0_m;
    constexpr auto AlgaeBargeArmFinish          = 0.0_deg;
}
#pragma endregion
