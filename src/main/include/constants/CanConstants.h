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

#pragma region CanConstants
namespace CanConstants
{
    const     auto CanBus                            = "rio";

    constexpr auto SwerveFrontLeftDriveMotorCanId    = 10;
    constexpr auto SwerveFrontLeftAngleMotorCanId    = 11;
    constexpr auto SwerveFrontLeftAngleEncoderCanId  = 20;

    constexpr auto SwerveFrontRightDriveMotorCanId   = 12;
    constexpr auto SwerveFrontRightAngleMotorCanId   = 13;
    constexpr auto SwerveFrontRightAngleEncoderCanId = 21;

    constexpr auto SwerveRearLeftDriveMotorCanId     = 14;
    constexpr auto SwerveRearLeftAngleMotorCanId     = 15;
    constexpr auto SwerveRearLeftAngleEncoderCanId   = 22;

    constexpr auto SwerveRearRightDriveMotorCanId    = 16;
    constexpr auto SwerveRearRightAngleMotorCanId    = 17;
    constexpr auto SwerveRearRightAngleEncoderCanId  = 23;

    constexpr auto ElevatorMotorCanId                = 33;
    constexpr auto ArmMotorCanId                     = 30;
    constexpr auto WristMotorCanId                   = 34;
    constexpr auto GripperMotorCanIdFixed            = 32;
    constexpr auto GripperMotorCanIdFree             = 31;

    constexpr auto ClimbMotorCanId                   = 40;

    constexpr auto MotorConfigurationAttempts        =  5;
}
#pragma endregion
