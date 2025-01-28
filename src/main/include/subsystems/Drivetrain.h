#pragma once

#include <numbers>

#include "studica/AHRS.h"

#include <hal/FRCUsageReporting.h>

#include <frc/ADIS16470_IMU.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc2/command/SubsystemBase.h>

#include "SwerveModule.h"

#include "Constants.h"

class Drivetrain : public frc2::SubsystemBase
{
    public:

        explicit        Drivetrain();

        void            Periodic() override;

        void            Drive(units::meters_per_second_t  xSpeed,
                              units::meters_per_second_t  ySpeed,
                              units::radians_per_second_t rotation);

        void            SetX();                           // Sets the wheels into an X formation to prevent movement

        void            ResetEncoders();                  // Resets the drive encoders to currently read a position of 0.
    
        void            SetModuleStates(wpi::array<frc::SwerveModuleState, ChassisConstants::NumberOfSwerveModules> desiredStates); 
        
        units::degree_t GetHeading();                     // Returns the heading of the robot.
        
        void            ZeroHeading();                    // Zeroes the heading of the robot.
      
        double          GetTurnRate();                    // Returns the turn rate of the robot.
      
        frc::Pose2d     GetPose();                        // Returns the currently-estimated pose of the robot.
      
        void            ResetOdometry(frc::Pose2d pose);  // Resets the odometry to the specified pose.

        void            SetFieldCentricity(bool fieldCentric);
        bool            GetFieldCentricity();

        void            SetWheelAnglesToZero();

        frc::SwerveDriveKinematics<ChassisConstants::NumberOfSwerveModules> m_kinematics{
            frc::Translation2d{ ChassisConstants::kWheelBase / 2,  ChassisConstants::kTrackWidth / 2},
            frc::Translation2d{ ChassisConstants::kWheelBase / 2, -ChassisConstants::kTrackWidth / 2},
            frc::Translation2d{-ChassisConstants::kWheelBase / 2,  ChassisConstants::kTrackWidth / 2},
            frc::Translation2d{-ChassisConstants::kWheelBase / 2, -ChassisConstants::kTrackWidth / 2}};

    private:

        studica::AHRS m_gyro{studica::AHRS::NavXComType::kMXP_SPI};  // The gyro sensor

        bool          m_fieldCentricity = false;                     // Field centricity flag

        SwerveModule  m_frontLeft;
        SwerveModule  m_rearLeft;
        SwerveModule  m_frontRight;
        SwerveModule  m_rearRight;

        // Odometry class for tracking robot pose for the swerve modules modules
        frc::SwerveDriveOdometry<ChassisConstants::NumberOfSwerveModules> m_odometry;
};
