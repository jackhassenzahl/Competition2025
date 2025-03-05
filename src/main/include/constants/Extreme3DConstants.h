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

#pragma region Extreme3DConstants
namespace Extreme3DConstants
{
    constexpr auto HandleTrigger    =  1;
    constexpr auto HandleSide       =  2;
    constexpr auto HandleLowerLeft  =  3;
    constexpr auto HandleLowerRight =  4;
    constexpr auto HandleUpperLeft  =  5;
    constexpr auto Handle7          =  7;
    constexpr auto Handle8          =  8;
    constexpr auto Handle9          =  9;
    constexpr auto Handle10         = 10;
    constexpr auto Handle11         = 11;
    constexpr auto Handle12         = 12;
    }
#pragma endregion
