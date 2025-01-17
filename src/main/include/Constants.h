#pragma once

#include <units/current.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

//#define ROBOT  // Enable code to run on the robot

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
    const std::string kCanBus = "rio";

    constexpr int kSwerveFrontRightDriveMotorCanId   = 1;
    constexpr int kSwerveFrontRightAngleMotorCanId   = 2;   
    constexpr int kSwerveFrontRightAngleEncoderCanId = 3;  

    constexpr int kSwerveFrontLeftDriveMotorCanId    = 4;
    constexpr int kSwerveFrontLeftAngleMotorCanId    = 5;   
    constexpr int kSwerveFrontLeftAngleEncoderCanId  = 6; 

    constexpr int kSwerveRearLeftDriveMotorCanId     = 7;
    constexpr int kSwerveRearLeftAngleMotorCanId     = 8;   
    constexpr int kSwerveRearLeftAngleEncoderCanId   = 9; 

    constexpr int kSwerveRearRightDriveMotorCanId    = 0;
    constexpr int kSwerveRearRightAngleMotorCanId    = 1;   
    constexpr int kSwerveRearRightAngleEncoderCanId  = 2;  
}

namespace ChassisConstants
{
    constexpr int                      kNumberOfSwerveModules           =     4;

    constexpr int                      kChassisLength                   =   100;
    constexpr int                      kChassisWidth                    =   100;

    constexpr units::current::ampere_t kSwerveDriveMaxAmperage          =  30_A;

    constexpr int                      kSwerveAngleMaxAmperage          =    30;

    constexpr int                      kSwerveWheelCountsPerRevoplution =    21;

    constexpr double                   kSwerveP                         = 0.700;      
    constexpr double                   kSwerveI                         = 0.000;   
    constexpr double                   kSwerveD                         = 0.001;
}

namespace JoystickConstants
{
    constexpr int    kJoystickDriverUsbPort   = 0;
    constexpr int    kJoystickOperatorUsbPort = 1;

    constexpr int    kJoystickForwardIndex    = 1;
    constexpr int    kJoystickStrifeIndex     = 0;
    constexpr int    kJoystickAngleIndex      = 2;  // 4 for xbox controller

    constexpr double kJoystickDeadZone        = 0.1;

    constexpr double kExponentForward         = 2.0;
    constexpr double kExponentStrife          = 2.0;
    constexpr double kExponentAngle           = 2.0;
}

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

namespace LedConstants
{
    constexpr int    kLength                =  40;  // The length of the LED string

    constexpr int    kPwmPort               =   5;

    constexpr double kBrightness            = 0.5;
    constexpr int    kRainbowRate           =   3;
    constexpr int    kMillisecondsToSeconds =  35;
}

namespace SwerveModuleConstants
{
    constexpr double kWheelRadius                  = 0.0508;
    constexpr int    kEncoderResolution            = 4096;
}