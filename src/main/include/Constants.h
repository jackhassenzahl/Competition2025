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

    constexpr auto ArmMotorCanId                     = 30;
    constexpr auto GrabberMotorCanId                 = 31;
    constexpr auto WristMotorCanId                   = 32;
    constexpr auto ElevatorMotorCanId                = 33;

    constexpr auto ClimbMotorCanId                   = 40;

    constexpr auto MotorConfigurationAttempts        =  5;
}
#pragma endregion

#pragma region DrivetrainConstants
namespace DrivetrainConstants
{
    // Chassis configuration
    constexpr auto TrackWidth                     = 0.6731_m;  // Distance between centers of right and left wheels on robot
    constexpr auto WheelBase                      = 0.6731_m;  // Distance between centers of front and back wheels on robot

    constexpr auto MaxSpeed                       = 4.8_mps;
    constexpr auto MaxAngularSpeed                = std::numbers::pi * 2_rad_per_s;

    constexpr auto UltraSonicPort                 =  0;
    constexpr auto UltraSonicSlope                =  0.2125;
    constexpr auto UltraSonicIntercept            = -2.1873;
}
#pragma endregion

#pragma region SwerveConstants
namespace SwerveConstants
{
    // Define the absolute encoder value for forward
    constexpr auto FrontRightForwardAngle          = -0.193359 * 2.0_rad * std::numbers::pi;
    constexpr auto FrontLeftForwardAngle           = -0.239014 * 2.0_rad * std::numbers::pi;
    constexpr auto RearRightForwardAngle           = -0.167772 * 2.0_rad * std::numbers::pi;
    constexpr auto RearLeftForwardAngle            =  0.259277 * 2.0_rad * std::numbers::pi;

    // Drive motor parameters
    constexpr auto DriveMaxAmperage                =     60_A;
    constexpr auto DriveMotorReduction             =     6.75;
    constexpr auto WheelDiameter                   = 0.1016_m;
    constexpr auto WheelCircumference              = WheelDiameter * std::numbers::pi;
    constexpr auto DriveMotorConversion            = WheelCircumference / DriveMotorReduction;

    constexpr auto DriveP                          = 0.10;
    constexpr auto DriveI                          = 0.02;
    constexpr auto DriveD                          = 0.00;
    constexpr auto DriveV                          = 0.10;
    constexpr auto DriveA                          = 0.10;

    // Angle motor parameters
    constexpr auto AngleMaxAmperage                =   20;
    constexpr auto AngleMotorRevolutions           = 21.5;  // The number of motor revolutions per wheel revolutions
    constexpr auto AngleRadiansToMotorRevolutions  = (2.0 * std::numbers::pi) / AngleMotorRevolutions;  // Radians to motor revolutions	

    constexpr auto AngleP                          = 1.00;
    constexpr auto AngleI                          = 0.00;
    constexpr auto AngleD                          = 0.20;
}
#pragma endregion

#pragma region ElevatorConstants
namespace ElevatorConstants
{
    constexpr auto P                               = 2.00;             // Proportional:    A position error of 0.2 rotations results in 12 V output
    constexpr auto I                               = 0.00;             // Integral:        No output for integrated error
    constexpr auto D                               = 0.10;             // Differential     A velocity error of 1 rps results in 0.5 V output
    constexpr auto S                               = 0.25;             // Static Friction: Add [voltage] output to overcome static friction
    constexpr auto V                               = 0.12;             // Velocity:        A velocity target of 1 rps results in [voltage] output
    constexpr auto A                               = 0.01;             // Acceleration:    An acceleration of 1 rps/s requires 0.01 V output

    constexpr auto MotionMagicCruiseVelocity       = 10_tps;           // Rotations per second cruise
    constexpr auto MotionMagicAcceleration         = 20_tr_per_s_sq;  // Acceleration
    constexpr auto MotionMagicJerk                 = 200_tr_per_s_cu;  // Jerk

    constexpr auto PositionToTurnsConversionFactor = 16.0 / (0.0475 * 3.0 * std::numbers::pi); // The number of motor rotations per meter
}
#pragma endregion

#pragma region GrabberConstants
namespace GrabberConstants
{
    constexpr auto WristP                          = 1.0;              // Proportional:    A position error of 0.2 rotations results in 12 V output
    constexpr auto WristI                          = 0.0;              // Integral:        No output for integrated error
    constexpr auto WristD                          = 0.1;              // Differential     A velocity error of 1 rps results in 0.5 V output

    constexpr auto GrabberP                        = 1.0;              // Proportional:    A position error of 0.2 rotations results in 12 V output
    constexpr auto GrabberI                        = 0.0;              // Integral:        No output for integrated error
    constexpr auto GrabberD                        = 0.1;              // Differential     A velocity error of 1 rps results in 0.5 V output

    constexpr auto GrabberMaxAmperage              =  60;
    constexpr auto WristMaxAmperage                =  60;

    constexpr auto WristRadiansToMotorRevolutions  = 2.0 * std::numbers::pi;  // Radians to motor revolutions	

    constexpr auto GrabberMaxRevolutionsPerMinute  = 180; // About 3 revolutions per second
}
#pragma endregion

#pragma region ArmConstants
namespace ArmConstants
{
    constexpr auto P                               = 1.0;              // Proportional:    A position error of 0.2 rotations results in 12 V output
    constexpr auto I                               = 0.0;              // Integral:        No output for integrated error
    constexpr auto D                               = 0.1;              // Differential     A velocity error of 1 rps results in 0.5 V output
    constexpr auto S                               = 0.0;              // Static Friction: Add [voltage] output to overcome static friction
    constexpr auto V                               = 0.0;              // Velocity:        A velocity target of 1 rps results in [voltage] output
    constexpr auto A                               = 0.0;              // Acceleration:    An acceleration of 1 rps/s requires 0.01 V output

    constexpr auto MotionMagicCruiseVelocity       = 110_tps;          // Rotations per second cruise
    constexpr auto MotionMagicAcceleration         = 500_tr_per_s_sq;  // Acceleration
    constexpr auto MotionMagicJerk                 = 500_tr_per_s_cu;  // Jerk

    constexpr auto AngleToTurnsConversionFactor    = 1.0;              // The number of motor rotations per degree
}
#pragma endregion

#pragma region ClimbConstants
namespace ClimbConstants
{
    constexpr auto P                               = 1.0;              // Proportional:    A position error of 0.2 rotations results in 12 V output
    constexpr auto I                               = 0.0;              // Integral:        No output for integrated error
    constexpr auto D                               = 0.1;              // Differential     A velocity error of 1 rps results in 0.5 V output
    constexpr auto S                               = 0.0;              // Static Friction: Add [voltage] output to overcome static friction
    constexpr auto V                               = 0.0;              // Velocity:        A velocity target of 1 rps results in [voltage] output
    constexpr auto A                               = 0.0;              // Acceleration:    An acceleration of 1 rps/s requires 0.01 V output

    constexpr auto MotionMagicCruiseVelocity       = 110_tps;          // Rotations per second cruise
    constexpr auto MotionMagicAcceleration         = 500_tr_per_s_sq;  // Acceleration
    constexpr auto MotionMagicJerk                 = 500_tr_per_s_cu;  // Jerk

    constexpr auto AngleToTurnsConversionFactor    =  100.0 / 16_deg;              // The number of motor rotations per degree

    constexpr auto MinClimbPosition                = -50_deg;         // Note: Need to calibrate angle to motor rotations
    constexpr auto MaxClimbPosition                = 100_deg;
}
#pragma endregion

#pragma region ControllerConstants
namespace ControllerConstants
{
    constexpr auto DriverControllerUsbPort =   0;
    constexpr auto JoystickOperatorUsbPort =   1;

    constexpr auto JoystickForwardIndex    =   1;
    constexpr auto JoystickStrafeIndex     =   0;
    constexpr auto JoystickAngleIndex      =   4;  // 4 for xbox controller, 2 for extreme 3d controller(stick controller)

    constexpr auto JoystickDeadZone        = 0.05;
    constexpr auto JoystickRotateDeadZone  = 0.15;

    constexpr auto ExponentForward         = 1.0;
    constexpr auto ExponentStrafe          = 1.0;
    constexpr auto ExponentAngle           = 1.0;
}
#pragma endregion

#pragma region Extreme3DConstants
namespace Extreme3DConstants
{
    constexpr auto HandleLowerLeft  = 3;
    constexpr auto HandleLowerRight = 4;
}
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

#pragma region ApriltagConstants
namespace ApriltagConstants
{
    // Magic camera values:
    // Source: https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21
    constexpr auto CameraWidthInPixels     = 699.3778103158814;
    constexpr auto CameraHeightInPixels    = 677.7161226393544;
    constexpr auto CameraCenterXInPixels   = 345.6059345433618;
    constexpr auto CameraCenterYInPixels   = 207.12741326228522;

    constexpr auto CameraResolutionWidth   = 640;
    constexpr auto CameraResolutionHeight  = 480;

    constexpr auto AprilTagLineWitdh       =   2;
    constexpr auto NumberOfAprilTagCorners =   4;
    constexpr auto NumberOfBitsCorrected   =   1;

    constexpr auto LengthOfTagsInches      = 6.5;

    constexpr auto RobotCameraOffset       = 0.0_m;
}
#pragma endregion

#pragma region LedConstants
namespace LedConstants
{
    constexpr auto Length      = 400;  // The length of the LED string
    constexpr auto PwmPort     =   9;
    constexpr auto Brightness  = 0.5;

    constexpr auto Red         = 255;
    constexpr auto Green       = 255;
    constexpr auto Blue        = 255;

    constexpr auto StrobeDelay =  20;  // The delay between strobe flashes
    constexpr auto HvaDelay    =  20;  // The delay between HVA color changes
}
#pragma endregion

#pragma region PoseConstants
namespace PoseConstants
{
    constexpr auto   MaxSpeed                   = 3_mps;
    constexpr auto   MaxAcceleration            = 3_mps_sq;
    constexpr auto   MaxAngularSpeed            = 3.142_rad_per_s;
    constexpr auto   MaxAngularAcceleration     = 3.142_rad_per_s_sq;

    constexpr double PXController               = 0.4;
    constexpr double PYController               = 0.4;
    constexpr double PProfileController         = 0.4;

    extern const frc::TrapezoidProfile<units::radians>::Constraints ThetaControllerConstraints;
}
#pragma endregion

#pragma region CoralPoseConstants
namespace CoralPoseConstants
{
    constexpr auto GroundArmAngle        = 0.0_deg;
    constexpr auto GroundElevator        = 0.0_m;
    constexpr auto GroundWristAngle      = 0.0_deg;
    constexpr auto GroundGrabberVelocity = 0.0;

    constexpr auto L1ArmAngle            = 0.0_deg;
    constexpr auto L1Elevator            = 0.0_m;
    constexpr auto L1WristAngle          = 0.0_deg;
    constexpr auto L1GrabberVelocity     = 0.0;

    constexpr auto L2ArmAngle            = 0.0_deg;
    constexpr auto L2Elevator            = 0.0_m;
    constexpr auto L2WristAngle          = 0.0_deg;
    constexpr auto L2GrabberVelocity     = 0.0;

    constexpr auto L3ArmAngle            = 0.0_deg;
    constexpr auto L3Elevator            = 0.0_m;
    constexpr auto L3WristAngle          = 0.0_deg;
    constexpr auto L3GrabberVelocity     = 0.0;

    constexpr auto L4ArmAngle            = 0.0_deg;
    constexpr auto L4Elevator            = 0.0_m;
    constexpr auto L4WristAngle          = 0.0_deg;
    constexpr auto L4GrabberVelocity     = 0.0;
}
#pragma endregion
