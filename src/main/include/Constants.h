#pragma once

#include <iostream>
#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Translation2d.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#pragma region CanConstants
//************************************************************************
// ***** Robio RIO Wiring Connections *****
//
// PWM Connections
//
//     PWM | Subsystem
//    -----+-----------
//       5 | LEDs
//
// CAN Identifications
//
//     ID | Subsystem       | Type                      | Type
//    ----+-----------------+---------------------------+------------------
//      1 | SwerveModule 0  | Front Right Drive Motor   |
//      2 | SwerveModule 0  | Front Right Angle Motor   |
//      3 | SwerveModule 0  | Front Right Angle Encoder |
//
//      4 | SwerveModule 1  | Front Left Drive Motor    |
//      5 | SwerveModule 1  | Front Left Angle Motor    |
//      6 | SwerveModule 1  | Front Left Angle Encoder  |
//
//      7 | SwerveModule 1  | Rear Left Drive Motor     |
//      8 | SwerveModule 1  | Rear Left Angle Motor     |
//      9 | SwerveModule 1  | Rear Left Angle Encoder   |
//
//     10 | SwerveModule 0  | Rear Right Drive Motor    |
//     11 | SwerveModule 0  | Rear Right Angle Motor    |
//     12 | SwerveModule 0  | Rear Right Angle Encoder  |
//
//************************************************************************

namespace CanConstants
{
    const std::string CanBus                        = "rio";

    constexpr int SwerveFrontRightDriveMotorCanId   =  3;
    constexpr int SwerveFrontRightAngleMotorCanId   =  2;
    constexpr int SwerveFrontRightAngleEncoderCanId = 12;

    constexpr int SwerveFrontLeftDriveMotorCanId    =  6;
    constexpr int SwerveFrontLeftAngleMotorCanId    =  5;
    constexpr int SwerveFrontLeftAngleEncoderCanId  =  4;

    constexpr int SwerveRearLeftDriveMotorCanId     =  9;
    constexpr int SwerveRearLeftAngleMotorCanId     =  8;
    constexpr int SwerveRearLeftAngleEncoderCanId   =  7;

    constexpr int SwerveRearRightDriveMotorCanId    = 11;
    constexpr int SwerveRearRightAngleMotorCanId    = 10;
    constexpr int SwerveRearRightAngleEncoderCanId  = 13;

    constexpr int ElevatorMotorCanId                =  2;  // TODO: Sharing with swerve drive motor for testing
}
#pragma endregion

#pragma region ChassisConstants
namespace ChassisConstants
{
    constexpr int                      NumberOfSwerveModules           =     4;

    constexpr int                      ChassisLength                   =   100;
    constexpr int                      ChassisWidth                    =   100;

    constexpr int                      MotorConfigurationAttempts      =     5;

    constexpr units::current::ampere_t SwerveDriveMaxAmperage          =  60_A;

    constexpr int                      SwerveAngleMaxAmperage          =    30;

    constexpr int                      SwerveWheelCountsPerRevoplution =    21;

    constexpr double                   SwerveP                         = 0.700;
    constexpr double                   SwerveI                         = 0.000;
    constexpr double                   SwerveD                         = 0.001;
}
#pragma endregion

#pragma region ControllerConstants
namespace ControllerConstants
{
    constexpr int    DriverControllerUsbPort = 0;
    constexpr int    JoystickOperatorUsbPort = 1;

    constexpr int    JoystickForwardIndex    = 1;
    constexpr int    JoystickStrafeIndex     = 0;
    constexpr int    JoystickAngleIndex      = 2;  // 4 for xbox controller

    constexpr double JoystickDeadZone        = 0.1;

    constexpr double ExponentForward         = 2.0;
    constexpr double ExponentStrafe          = 2.0;
    constexpr double ExponentAngle           = 2.0;
}
#pragma endregion

#pragma region ApriltagConstants
namespace ApriltagConstants
{
    // Magic camera values:
    // Source: https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21
    constexpr double CameraWidthInPixels     = 699.3778103158814;
    constexpr double CameraHeightInPixels    = 677.7161226393544;
    constexpr double CameraCenterXInPixels   = 345.6059345433618;
    constexpr double CameraCenterYInPixels   = 207.12741326228522;

    constexpr int    CameraResolutionWidth   = 640;
    constexpr int    CameraResolutionHeight  = 480;

    constexpr int    AprilTagLineWitdh       =   2;
    constexpr int    NumberOfAprilTagCorners =   4;
    constexpr int    NumberOfBitsCorrected   =   1;

    constexpr double LengthOfTagsInches      = 6.5;
}
#pragma endregion

#pragma region Extreme3DContants
namespace Extreme3DContants
{
    constexpr int HandleLowerLeft  = 3;
    constexpr int HandleLowerRight = 4;
}
#pragma endregion

#pragma region xBoxConstants
namespace XBoxConstants
{
    constexpr int    A                 =   1;
    constexpr int    B                 =   2;
    constexpr int    X                 =   3;
    constexpr int    Y                 =   4;
    constexpr int    LeftBumper        =   5;
    constexpr int    RightBumper       =   6;
    constexpr int    Back              =   7;
    constexpr int    Start             =   8;
    constexpr int    LeftStickButton   =   9;
    constexpr int    RightStickButton  =  10;
 
    constexpr int    Pov_0             =   0;
    constexpr int    Pov_45            =  45;
    constexpr int    Pov_90            =  90;
    constexpr int    Pov_135           = 135;
    constexpr int    Pov_180           = 180;
    constexpr int    Pov_225           = 225;
    constexpr int    Pov_270           = 270;
    constexpr int    Pov_315           = 315;
}
#pragma endregion

#pragma region LedConstants
namespace LedConstants
{
    constexpr int    Length      =  40;  // The length of the LED string
    constexpr int    PwmPort     =   5;
    constexpr double Brightness  = 0.5;

    constexpr int    Red         = 255;
    constexpr int    Green       = 255;
    constexpr int    Blue        = 255;

    constexpr int    StrobeDelay =  20;  // The delay between strobe flashes
    constexpr int    HvaDelay    =  20;  // The delay between HVA color changes
}
#pragma endregion
