#pragma once

// ***** Robio RIO Connections *****

// CAN Identifications
#define SWERVE_FRONT_RIGHT_DRIVE_MOTOR_CAN_ID    1
#define SWERVE_FRONT_RIGHT_ANGLE_MOTOR_CAN_ID    2
#define SWERVE_FRONT_RIGHT_ANGLE_ENCODER_CAN_ID  3

#define SWERVE_FRONT_LEFT_DRIVE_MOTOR_CAN_ID     4
#define SWERVE_FRONT_LEFT_ANGLE_MOTOR_CAN_ID     5
#define SWERVE_FRONT_LEFT_ANGLE_ENCODER_CAN_ID   6

#define SWERVE_REAR_LEFT_DRIVE_MOTOR_CAN_ID      7
#define SWERVE_REAR_LEFT_ANGLE_MOTOR_CAN_ID      8
#define SWERVE_REAR_LEFT_ANGLE_ENCODER_CAN_ID    9

#define SWERVE_REAR_RIGHT_DRIVE_MOTOR_CAN_ID    10
#define SWERVE_REAR_RIGHT_ANGLE_MOTOR_CAN_ID    11
#define SWERVE_REAR_RIGHT_ANGLE_ENCODER_CAN_ID  12

// PWMs
#define LED_PWM_PORT                             5

// ***** Driver Station Connections *****
#define JOY_STICK_DRIVER_USB_PORT                0
#define JOY_STICK_CONTROLLER_USB_PORT            1

// ***** April Tag Processing *****
// Magic camera values:
// Source: https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21
#define CAMERA_WIDTH_IN_PIXELS                 699.3778103158814
#define CAMERA_HEIGHT_IN_PIXELS                677.7161226393544
#define CAMERA_CENTER_X_IN_PIXELS              345.6059345433618
#define CAMERA_CENTER_Y_IN_PIXELS              207.12741326228522

#define CAMERA_RESOLUTION_WIDTH                640
#define CAMERA_RESOLUTION_HEIGTH               480

#define APRIL_TAG_LINE_WIDTH                     2
#define NUMBER_OF_APRIL_TAG_CORNER               4
#define NUMBER_OF_BITS_CORRECTED                 1
#define LENGTH_OF_TAGS_INCHES                    6.5

// ***** Chassis Definitions *****
#define NUMBER_OF_SWERVE_MODULES                 4

#define CHASSIS_LENGTH                         100
#define CHASSIS_WIDTH                          100

// ***** LED Controls *****
#define BRIGHTNESS                             0.5
#define RAINBOW_RATE                             3
#define MILLISECONDS_TO_SECONDS                 35