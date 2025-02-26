#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>

#include "Constants.h"

/// @brief modes for the LED string.
enum GripperPoseEnum
{
    CoralGround,
    CoralStation,
    CoralL1,
    CoralL2,
    CoralL3,
    CoralL4,

    AlgaeGround,
    AlgaeOnCoral,
    AlgaeLo,
    AlgaeHigh,
    AlgaeProcessor,
    AlgaeBarge
};

class Gripper : public frc2::SubsystemBase
{
    public:

        Gripper();

        void                   SetPose(GripperPoseEnum pose);

        void                   SetElevatorHeight(units::length::meter_t position);
        void                   SetElevatorOffset(units::length::meter_t offset);

        void                   SetArmAngle(units::angle::degree_t angle);
        units::angle::degree_t GetArmAngle();

        void                   SetArmAngleOffset(units::angle::degree_t offset);

        void                   SetWristAngle(units::angle::degree_t angle);
        units::angle::degree_t GetWristAngle();

        void                   SetGripperWheelsVoltage(units::voltage::volt_t voltage);

        GripperPoseEnum        GetPose() { return m_pose; }  // Get the Gripper Pose

    private:

        void ConfigureElevatorMotor(int driveMotorCanId);
        void ConfigureArmMotor(int driveMotorCanId);
        void ConfigureWristMotor();
        void ConfigureGripperMotorRight();
        void ConfigureGripperMotorLeft();

        ctre::phoenix6::hardware::TalonFX           *m_elevatorMotor;
        ctre::phoenix6::controls::MotionMagicVoltage m_elevatorMotionMagicVoltage{0_tr};

        ctre::phoenix6::hardware::TalonFX           *m_armMotor;
        ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicVoltage{0_tr};

        rev::spark::SparkMax                         m_wristMotor;
        rev::spark::SparkClosedLoopController        m_wristTurnClosedLoopController;
        rev::spark::SparkRelativeEncoder             m_wristEncoder;

        rev::spark::SparkMax                         m_gripperMotorRight;
        rev::spark::SparkMax                         m_gripperMotorLeft;

        GripperPoseEnum                              m_pose;
};
