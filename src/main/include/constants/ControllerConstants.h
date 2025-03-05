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

#pragma region ControllerConstants
namespace ControllerConstants
{
    constexpr auto DriverControllerUsbPort =   0;
    constexpr auto JoystickOperatorUsbPort =   1;

    constexpr auto JoystickForwardIndex    =   1;
    constexpr auto JoystickStrafeIndex     =   0;
    constexpr auto JoystickAngleIndex      =   2;  // 4 for xbox controller, 2 for extreme 3d controller(stick controller)

    constexpr auto JoystickDeadZone        = 0.05;
    constexpr auto JoystickRotateDeadZone  = 0.15;

    constexpr auto ExponentForward         = 1.0;
    constexpr auto ExponentStrafe          = 1.0;
    constexpr auto ExponentAngle           = 5.0;
}
#pragma endregion
