#pragma once

#include <numbers>

#include <rev/SparkMax.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/PWMSparkMax.h>

#include <cmath>

#include <rev/SparkMax.h>
#include <ctre/phoenix6/configs/Configs.hpp>

#include <frc/geometry/Rotation2d.h>


#include "Constants.h"

class SwerveModule
{
    public:

        explicit                   SwerveModule(int driveMotorCANid, int angleMotorCANid, int angleEncoderCANid);

        void                       SetState(frc::SwerveModuleState &state);

        frc::SwerveModuleState     GetState();
        frc::SwerveModulePosition  GetPosition();

    private:

        void                       ConfigureDriveMotor(int driveMotorCANid);
        void                       ConfigureAngleMotor(int angleMotorCANid, int angleEncoderCANid);

        units::meters_per_second_t GetDriveEncoderRate();
        units::radian_t            GetAngleEncoderDistance();

        static constexpr auto kModuleMaxAngularVelocity     = std::numbers::pi * 1_rad_per_s;       // radians per second
        static constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s / 1_s; // radians per second^2

        // Swerve drive motor
        // Note: Encoder is built in the TalonFX
        ctre::phoenix6::hardware::TalonFX          *m_driveMotor;
        
        // Swerve angle motor, encoder and PID controller
        rev::spark::SparkMax                       *m_angleMotor;
        rev::spark::SparkRelativeEncoder           *m_angleEncoder;
        ctre::phoenix6::hardware::CANcoder         *m_angleAbsoluteEncoder;
        
        frc::PIDController                          m_drivePIDController{1.0, 0, 0};
        frc::ProfiledPIDController<units::radians>  m_turningPIDController{1.0, 0.0, 0.0, {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

        frc::SimpleMotorFeedforward<units::meters>  m_driveFeedforward{1_V,   3_V / 1_mps};
        frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward {1_V, 0.5_V / 1_rad_per_s};
};
