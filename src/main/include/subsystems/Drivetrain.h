#pragma once

#include <cmath>
#include <numbers>

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include "Constants.h"
#include "SwerveModule.h"

class Drivetrain : public frc2::SubsystemBase
{
    public:

        explicit Drivetrain() { m_gyro.Reset(); }

        void     Drive(units::meters_per_second_t  xSpeed,
                       units::meters_per_second_t  ySpeed,
                       units::radians_per_second_t rot,
                       bool fieldRelative, 
                       units::second_t period);

        void     UpdateOdometry();

        void     SetFieldCentricity(bool fieldCentric);
        bool     GetFieldCentricity();

        static constexpr units::meters_per_second_t  kMaxSpeed = 3.0_mps; // 3 meters per second
        static constexpr units::radians_per_second_t kMaxAngularSpeed{std::numbers::pi}; // 1/2 rotation per second

    private:

        bool   m_fieldCentricity = false;

        frc::Translation2d m_frontLeftLocation {+0.381_m, +0.381_m};
        frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
        frc::Translation2d m_backLeftLocation  {-0.381_m, +0.381_m};
        frc::Translation2d m_backRightLocation {-0.381_m, -0.381_m};

        /// Swerve Module Indexes:
        ///
        ///          Front
        ///       +---------+ ---
        ///       |[1]   [0]|  ^       0   Front Right
        ///       |         |  |       1   Front Left
        ///       |         | Length   2   Rear Left
        ///       |         |  |       3   Rear Right
        ///       |[2]   [3]|  v
        ///       +---------+ ---
        ///       |         |
        ///       |< Width >|
        SwerveModule       m_frontLeft {CanConstants::kSwerveFrontRightDriveMotorCanId, 
                                        CanConstants::kSwerveFrontRightAngleMotorCanId, 
                                        CanConstants::kSwerveFrontRightAngleEncoderCanId};
        SwerveModule       m_frontRight{CanConstants::kSwerveFrontLeftDriveMotorCanId, 
                                        CanConstants::kSwerveFrontLeftAngleMotorCanId, 
                                        CanConstants::kSwerveFrontLeftAngleEncoderCanId};
        SwerveModule       m_backLeft  {CanConstants::kSwerveRearLeftDriveMotorCanId, 
                                        CanConstants::kSwerveRearLeftAngleMotorCanId, 
                                        CanConstants::kSwerveRearLeftAngleEncoderCanId};
        SwerveModule       m_backRight {CanConstants::kSwerveRearRightDriveMotorCanId, 
                                        CanConstants::kSwerveRearRightAngleMotorCanId, 
                                        CanConstants::kSwerveRearRightAngleEncoderCanId};
    
        frc::AnalogGyro    m_gyro{0};
    
        frc::SwerveDriveKinematics<4> m_kinematics{m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation};
    
        frc::SwerveDriveOdometry<4>   m_odometry{m_kinematics, m_gyro.GetRotation2d(), {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                                                 m_backLeft.GetPosition(), m_backRight.GetPosition()}};
};
