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
    constexpr auto GripperMotorCanIdRight            = 31;
    constexpr auto GripperMotorCanIdLeft             = 32;

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
    constexpr auto FrontRightForwardAngle          = -0.193604 * 2.0_rad * std::numbers::pi;
    constexpr auto FrontLeftForwardAngle           = -0.422119 * 2.0_rad * std::numbers::pi;
    constexpr auto RearRightForwardAngle           = -0.174561 * 2.0_rad * std::numbers::pi;
    constexpr auto RearLeftForwardAngle            =  0.268555 * 2.0_rad * std::numbers::pi;

    // Drive motor parameters
    constexpr auto DriveMaxAmperage                =     60_A;
    constexpr auto DriveMotorReduction             =     6.75;
    constexpr auto WheelDiameter                   = 0.0961_m;
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

    constexpr auto MotionMagicCruiseVelocity       = 40_tps;           // Rotations per second cruise
    constexpr auto MotionMagicAcceleration         = 80_tr_per_s_sq;  // Acceleration
    constexpr auto MotionMagicJerk                 = 800_tr_per_s_cu;  // Jerk

    constexpr auto PositionToTurnsConversionFactor = 64.0 / (0.06378 * 3.0 * std::numbers::pi); // The number of motor rotations per meter

    constexpr auto MinimumPosition                 = 0_m;
    constexpr auto MaximumPosition                 = 2.0_m;  // TODO: Determine the maximum height

    constexpr auto HeightOffset                    = 0.1_m;
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

    constexpr auto AngleToTurnsConversionFactor    = 360_deg / 36;      // 36 to 1 gear box

    constexpr auto AngleMaxAmperage                = 20;

    constexpr auto MinimumPosition                 = -50_deg;          // Note: Need to calibrate angle to motor rotations
    constexpr auto MaximumPosition                 = 100_deg;

    constexpr auto AngleOffset                     = 5_deg;
}
#pragma endregion

#pragma region WristConstants
namespace WristConstants
{
    constexpr auto P                             = 1.0;              // Proportional:    A position error of 0.2 rotations results in 12 V output
    constexpr auto I                             = 0.0;              // Integral:        No output for integrated error
    constexpr auto D                             = 0.1;              // Differential     A velocity error of 1 rps results in 0.5 V output

    constexpr auto MaxAmperage                   =  60;
    constexpr auto AngleToTurnsConversionFactor  = 360_deg / 20;     // 20 to 1 gear box
    constexpr auto AngleOffset                   = 5_deg;
}
#pragma endregion

#pragma region GripperConstants
namespace GripperConstants
{
    constexpr auto GripperP           = 1.0;              // Proportional:    A position error of 0.2 rotations results in 12 V output
    constexpr auto GripperI           = 0.0;              // Integral:        No output for integrated error
    constexpr auto GripperD           = 0.1;              // Differential     A velocity error of 1 rps results in 0.5 V output

    constexpr auto GripperMaxAmperage =  60;
}
#pragma endregion

#pragma region ClimbConstants
namespace ClimbConstants
{
    constexpr auto ClimbVoltage       = 12_V;

    constexpr auto ClimbLimitSwtich   = 0;
    constexpr auto CaptureLimitSwitch = 1;
}
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
    constexpr auto JoystickRotateDeadZone  = 0.3;

    constexpr auto ExponentForward         = 1.0;
    constexpr auto ExponentStrafe          = 1.0;
    constexpr auto ExponentAngle           = 5.0;
}
#pragma endregion

#pragma region Extreme3DConstants
namespace Extreme3DConstants
{
    constexpr auto HandleTrigger    = 1;
    constexpr auto HandleSide       = 2;
    constexpr auto HandleLowerLeft  = 3;
    constexpr auto HandleLowerRight = 4;
    constexpr auto HandleUpperLeft  = 5;
    }
#pragma endregion

#pragma region ControlPanelConstants
namespace ControlPanelConstants
{
    constexpr auto CoralGnd        =  0;
    constexpr auto CoralStn        =  5;
    constexpr auto CoralL1         =  1;
    constexpr auto CoralL2         =  6;
    constexpr auto CoralL3         =  2;
    constexpr auto CoralL4         = 15;
    constexpr auto CoralSideSelect = 14;

    constexpr auto AlgaeGnd        = 17;
    constexpr auto AlgaeCoral      = 19;
    constexpr auto AlgaeLow        = 16;
    constexpr auto AlgaeHigh       = 18;
    constexpr auto AlgaeProcessor  =  4;
    constexpr auto AlgaeBarge      =  3;

    constexpr auto Activate        = 12;

    constexpr auto Toggle          = 10;

    constexpr auto ElevatorUp      =  9;
    constexpr auto ElevatorDown    = 11;

    constexpr auto ClimbUp         =  7;
    constexpr auto ClimbDown       = 13;

    constexpr auto GripperMotor    =  5;

    constexpr auto Spare           =  8;
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

    constexpr auto RobotCameraOffset       = -0.305_m;
}
#pragma endregion

#pragma region LedConstants
namespace LedConstants
{
    constexpr auto PwmPort     =   9;

    constexpr auto Length      = 400;  // The length of the LED string
    constexpr auto Brightness  = 0.5;

    constexpr auto Red         = 255;
    constexpr auto Green       = 255;
    constexpr auto Blue        = 255;

    constexpr auto StrobeDelay =  20;  // The delay between strobe flashes
    constexpr auto HvaDelay    =  20;  // The delay between HVA color changes
}
#pragma endregion

#pragma region ChassisPoseConstants
namespace ChassisPoseConstants
{
    constexpr auto   MaxSpeed                   = 3_mps;
    constexpr auto   MaxAcceleration            = 3_mps_sq;
    constexpr auto   MaxAngularSpeed            = 3.142_rad_per_s;
    constexpr auto   MaxAngularAcceleration     = 3.142_rad_per_s_sq;

    constexpr double PXController               = 4.0;
    constexpr double PYController               = 4.0;
    constexpr double PProfileController         = 5.0;

    extern const frc::TrapezoidProfile<units::radians>::Constraints ThetaControllerConstraints;
}
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
    constexpr auto Coral123ElevatorOffset       = 0.1_m;
    constexpr auto Coral123Wait1                = 0.1_s;
    constexpr auto Coral123ArmOffset            = 0.1_deg;
    constexpr auto Coral123Wait2                = 0.1_s;
    constexpr auto Coral123GripperVoltage       = 0.1_V;
    constexpr auto Coral123Wait3                = 0.1_s;
    constexpr auto Coral123ElevatorFinish       = 0.1_m;
    constexpr auto Coral123ArmFinish            = 0.1_deg;

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

#pragma region CoralPoseConstants
namespace CoralPoseConstants
{
    constexpr auto GroundElevator         = 0.0_m;
    constexpr auto GroundArmAngle         = 0.0_deg;
    constexpr auto GroundWristAngle       = 0.0_deg;
    constexpr auto GroundGripperVoltage   = 0.0_V;

    constexpr auto StationElevator        = 0.5_m;
    constexpr auto StationArmAngle        = 45.0_deg;
    constexpr auto StationWristAngle      = 0.0_deg;
    constexpr auto StationGripperVoltage  = 0.0_V;

    constexpr auto L1Elevator             = 0.5_m;
    constexpr auto L1ArmAngle             = 45.0_deg;
    constexpr auto L1WristAngle           = 90.0_deg;
    constexpr auto L1GripperVoltage       = 0.0_V;

    constexpr auto L2Elevator             = 1.0_m;
    constexpr auto L2ArmAngle             = 50.0_deg;
    constexpr auto L2WristAngle           = 90.0_deg;
    constexpr auto L2GripperVoltage       = 0.0_V;

    constexpr auto L3Elevator             = 1.5_m;
    constexpr auto L3ArmAngle             = 55.0_deg;
    constexpr auto L3WristAngle           = 90.0_deg;
    constexpr auto L3GripperVoltage       = 1.0_V;

    constexpr auto L4Elevator             = 1.5_m;
    constexpr auto L4ArmAngle             = 60.0_deg;
    constexpr auto L4WristAngle           = 90.0_deg;
    constexpr auto L4GripperVoltage       = -1.0_V;
}
#pragma endregion

#pragma region AlgaePoseConstants
namespace AlgaePoseConstants
{
    constexpr auto GroundElevator          = 0.0_m;
    constexpr auto GroundArmAngle          = 0.0_deg;
    constexpr auto GroundWristAngle        = 0.0_deg;
    constexpr auto GroundGripperVoltage    = 0.0_V;

    constexpr auto OnCoralElevator         = 0.0_m;
    constexpr auto OnCoralArmAngle         = 0.0_deg;
    constexpr auto OnCoralWristAngle       = 0.0_deg;
    constexpr auto OnCoralGripperVoltage   = 0.0_V;

    constexpr auto LoElevator              = 0.0_m;
    constexpr auto LoArmAngle              = 0.0_deg;
    constexpr auto LoWristAngle            = 0.0_deg;
    constexpr auto LoGripperVoltage        = 0.0_V;

    constexpr auto HighElevator            = 0.0_m;
    constexpr auto HighArmAngle            = 0.0_deg;
    constexpr auto HighWristAngle          = 0.0_deg;
    constexpr auto HighGripperVoltage      = 0.0_V;

    constexpr auto ProcessorElevator       = 0.0_m;
    constexpr auto ProcessorArmAngle       = 0.0_deg;
    constexpr auto ProcessorWristAngle     = 0.0_deg;
    constexpr auto ProcessorGripperVoltage = 0.0_V;

    constexpr auto BargeElevator           = 0.0_m;
    constexpr auto BargeArmAngle           = 0.0_deg;
    constexpr auto BargeWristAngle         = 0.0_deg;
    constexpr auto BargeGripperVoltage     = 0.0_V;
}
#pragma endregion

#pragma region AutonomousConstants
namespace AutonomousConstants
{
    constexpr auto OneCoralSpeed            = 1_mps;
    constexpr auto OneCoralXDistance        = 1_m;
    constexpr auto OneCoralYDistance        = 1_m;
    constexpr auto OneCoralAngleChange      = 0_deg;
    constexpr auto OneCoralTimeOut          = 10_s;

    constexpr auto AlgaeAndCoralSpeed       = 1_mps;
    constexpr auto AlgaeAndCoralXDistance   = -1_m;
    constexpr auto AlgaeAndCoralYDistance   = 0_m;
    constexpr auto AlgaeAndCoralAngleChange = 180_deg;
    constexpr auto AlgaeAndCoralTimeOut     = 10_s;
};
#pragma endregion
