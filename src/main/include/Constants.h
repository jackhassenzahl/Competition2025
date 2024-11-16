#pragma once

// ***** Robio RIO Connections *****

// CAN Identifications

// PWMs
#define LED_PWM_PORT               5

// ***** Driver Station Connections *****
#define JoyStickDriverUsbPort      0
#define JoyStickControllerUsbPort  1

// ***** April Tag Processing *****
// Magic camera values:
// Source: https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21
#define CameraWidthInPixels      699.3778103158814
#define CameraHightInPixels      677.7161226393544
#define CameraCenterXInPixels    345.6059345433618
#define CameraCenterYInPixels    207.12741326228522

#define CameraResolutionWidth    640
#define CameraResolutionHeight   480

#define AprilTagLineWidth          2
#define NumberOfAprilTagCorners    4
#define NumOfBitsCorrected         1
#define LengthOfTagsInches         6.5


// ***** LED Controls *****
#define BRIGHTNESS               0.5
#define RAINBOW_RATE               3

// ***** Chassis Definitions *****
#define NumberOfSwerveModules      4

#define ChassisLength            100
#define ChassisWidth             100
