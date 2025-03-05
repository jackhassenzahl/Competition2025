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

#pragma region xBoxConstants
namespace XBoxConstants
{
    constexpr auto A                 =   1;
    constexpr auto B                 =   2;
    constexpr auto X                 =   3;
    constexpr auto Y                 =   4;
    constexpr auto LeftBumper        =   5;
    constexpr auto RightBumper       =   6;
    constexpr auto Back              =   7;
    constexpr auto Start             =   8;
    constexpr auto LeftStickButton   =   9;
    constexpr auto RightStickButton  =  10;

    constexpr auto Pov_0             =   0;
    constexpr auto Pov_45            =  45;
    constexpr auto Pov_90            =  90;
    constexpr auto Pov_135           = 135;
    constexpr auto Pov_180           = 180;
    constexpr auto Pov_225           = 225;
    constexpr auto Pov_270           = 270;
    constexpr auto Pov_315           = 315;
}
#pragma endregion
 