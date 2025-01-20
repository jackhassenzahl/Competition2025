#pragma once

#include <numbers>

#include <frc/AnalogGyro.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc2/command/SubsystemBase.h>

#include "SwerveModule.h"

#include "Constants.h"

class Drivetrain : public frc2::SubsystemBase
{
    public:

        explicit Drivetrain() { m_gyro.Reset(); }

        void     Drive(units::meters_per_second_t  xSpeed,
                       units::meters_per_second_t  ySpeed,
                       units::radians_per_second_t rotation,
                       units::second_t             period);

        void     UpdateOdometry();

        void     SetFieldCentricity(bool fieldCentric);
        bool     GetFieldCentricity();

        static constexpr units::meters_per_second_t  kMaxSpeed{3.0_mps};                 // 3 meters per second
        static constexpr units::radians_per_second_t kMaxAngularSpeed{std::numbers::pi}; // 1/2 rotation per second

    private:

        bool               m_fieldCentricity = false;

        frc::Translation2d m_frontLeftLocation {+0.381_m, +0.381_m};
        frc::Translation2d m_frontRightLocation{+0.381_m, -0.381_m};
        frc::Translation2d m_backLeftLocation  {-0.381_m, +0.381_m};
        frc::Translation2d m_backRightLocation {-0.381_m, -0.381_m};

        /// Swerve Module Indexes:
        ///
        ///          Front
        ///       +---------+ ---
        ///       |[0]   [1]|  ^       0   Front Left
        ///       |         |  |       1   Front Right
        ///       |         | Length   2   Rear Left
        ///       |         |  |       3   Rear Right
        ///       |[2]   [3]|  v
        ///       +---------+ ---
        ///       |         |
        ///       |< Width >|
        SwerveModule       m_frontLeft {CanConstants::SwerveFrontRightDriveMotorCanId, 
                                        CanConstants::SwerveFrontRightAngleMotorCanId, 
                                        CanConstants::SwerveFrontRightAngleEncoderCanId};
        SwerveModule       m_frontRight{CanConstants::SwerveFrontLeftDriveMotorCanId, 
                                        CanConstants::SwerveFrontLeftAngleMotorCanId, 
                                        CanConstants::SwerveFrontLeftAngleEncoderCanId};
        SwerveModule       m_backLeft  {CanConstants::SwerveRearLeftDriveMotorCanId, 
                                        CanConstants::SwerveRearLeftAngleMotorCanId, 
                                        CanConstants::SwerveRearLeftAngleEncoderCanId};
        SwerveModule       m_backRight {CanConstants::SwerveRearRightDriveMotorCanId, 
                                        CanConstants::SwerveRearRightAngleMotorCanId, 
                                        CanConstants::SwerveRearRightAngleEncoderCanId};
    
        frc::AnalogGyro    m_gyro{0};
    
        frc::SwerveDriveKinematics<ChassisConstants::NumberOfSwerveModules> m_kinematics{m_frontLeftLocation, m_frontRightLocation,
                                                                                         m_backLeftLocation, m_backRightLocation};
    
        frc::SwerveDriveOdometry<ChassisConstants::NumberOfSwerveModules>   m_odometry{m_kinematics, m_gyro.GetRotation2d(), 
                                                                                      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                                                                                       m_backLeft.GetPosition(),  m_backRight.GetPosition()}};
};
