#pragma once

#include <numbers>
#include <cmath>
#include "string.h"

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <rev/SparkMax.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class SwerveModule
{
    public:

        explicit                  SwerveModule(int driveMotorCanId, int angleMotorCanId, int angleEncoderCanId, double chassisAngularOffset);

        frc::SwerveModuleState    GetState();                                            // Returns the current state of the module

        frc::SwerveModulePosition GetPosition();                                         // Returns the current position of the module
      
        void                      SetDesiredState(const frc::SwerveModuleState& state);  // Sets the desired state for the module
        
        void                      ResetEncoders();                                       // Zeroes all the  encoders

        void                      SetWheelAngleToForward(units::angle::degree_t desiredAngle);

        void                      SetState(frc::SwerveModuleState &state, std::string description);

    private:

        // Private methods
        void                       ConfigureDriveMotor();
        void                       ConfigureAngleMotor();

        units::angle::degree_t     GetAbsoluteAngle();
        units::meters_per_second_t GetDriveEncoderRate();
        units::radian_t            GetAngleEncoderDistance();

        static constexpr auto kModuleMaxAngularVelocity     = std::numbers::pi * 1_rad_per_s;       // radians per second
        static constexpr auto kModuleMaxAngularAcceleration = std::numbers::pi * 2_rad_per_s / 1_s; // radians per second^2

        ctre::phoenix6::hardware::TalonFX           m_driveMotor;
        rev::spark::SparkMax                        m_angleMotor;

        rev::spark::SparkAbsoluteEncoder            m_turnAbsoluteEncoder  = m_angleMotor.GetAbsoluteEncoder();

        double                                      m_chassisAngularOffset = 0;

        frc::SwerveModuleState                      m_desiredState{units::meters_per_second_t{0.0}, frc::Rotation2d()};  


        rev::spark::SparkRelativeEncoder            m_angleEncoder;
        ctre::phoenix6::hardware::CANcoder          m_angleAbsoluteEncoder;

        frc::PIDController                          m_drivePIDController{1.0, 0, 0};
        frc::ProfiledPIDController<units::radians>  m_turningPIDController{1.0, 0.0, 0.0, {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

        frc::SimpleMotorFeedforward<units::meters>  m_driveFeedforward{1_V,   3_V / 1_mps};
        frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward {1_V, 0.5_V / 1_rad_per_s};
};
