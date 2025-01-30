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

        void            SetX();                                 // Sets the wheels into an X formation to prevent movement

        void            ResetDriveEncoders();                   // Resets the drive encoders to currently read a position of 0.

        void            SetModuleStates(wpi::array<frc::SwerveModuleState, ChassisConstants::NumberOfSwerveModules> desiredStates);

        units::radian_t GetHeading();                           // Returns the heading of the robot.

        void            ZeroHeading();                          // Zeroes the heading of the robot.

        frc::Pose2d     GetPose();                              // Returns the currently-estimated pose of the robot.

        void            ResetOdometry(frc::Pose2d pose);        // Resets the odometry to the specified pose.

        void            SetFieldCentricity(bool fieldCentric);  // Sets the field centricity
        bool            GetFieldCentricity();                   // Reads the field centricity

        void            SetWheelAnglesToZero();                 // Sets the wheels to forward based on the absolute encoder

        frc::SwerveDriveKinematics<ChassisConstants::NumberOfSwerveModules> m_kinematics{
            frc::Translation2d{ ChassisConstants::WheelBase / 2,  ChassisConstants::TrackWidth / 2},
            frc::Translation2d{ ChassisConstants::WheelBase / 2, -ChassisConstants::TrackWidth / 2},
            frc::Translation2d{-ChassisConstants::WheelBase / 2,  ChassisConstants::TrackWidth / 2},
            frc::Translation2d{-ChassisConstants::WheelBase / 2, -ChassisConstants::TrackWidth / 2}};

    private:

        studica::AHRS m_gyro{studica::AHRS::NavXComType::kMXP_SPI};  // The gyro sensor

        bool          m_fieldCentricity = false;                     // Field centricity flag

        SwerveModule  m_frontLeft;
        SwerveModule  m_frontRight;
        SwerveModule  m_rearLeft;
        SwerveModule  m_rearRight;

        // Odometry class for tracking robot pose for the swerve modules modules
        frc::SwerveDriveOdometry<ChassisConstants::NumberOfSwerveModules> m_odometry;
};
