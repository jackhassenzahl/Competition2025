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

#pragma region ControlPanelConstants
namespace ControlPanelConstants
{
    // Digital Inputs
    constexpr auto CoralGnd        =  1;
    constexpr auto CoralStn        =  6;
    constexpr auto CoralL1         =  2;
    constexpr auto CoralL2         =  7;
    constexpr auto CoralL3         =  3;
    constexpr auto CoralL4         = 16;
    constexpr auto CoralSideSelect = 15;

    constexpr auto AlgaeGnd        = 18;
    constexpr auto AlgaeCoral      = 20;
    constexpr auto AlgaeLow        = 17;
    constexpr auto AlgaeHigh       = 19;
    constexpr auto AlgaeProcessor  =  5;
    constexpr auto AlgaeBarge      =  4;

    constexpr auto Activate        = 13;

    constexpr auto Toggle          = 11;

    constexpr auto ElevatorUp      = 10;
    constexpr auto ElevatorDown    = 12;

    constexpr auto ClimbUp         = 14;
    constexpr auto ClimbDown       =  8;

    constexpr auto Spare           =  9;

    // Analog Inputs
    constexpr auto GripperMotor    =  3;
}
#pragma endregion
