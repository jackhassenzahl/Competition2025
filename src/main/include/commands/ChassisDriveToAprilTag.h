#pragma once

#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/AprilTags.h"
#include "subsystems/Drivetrain.h"

struct ChassDriveAprilTagParameters
{
    bool                       ValidPose;
    bool                       ReefRightSide;
    units::meters_per_second_t Speed;
    units::meter_t             DistanceOffsetX;
    units::meter_t             DistanceOffsetY;
    units::degree_t            AngleOffset;
    units::time::second_t      TimeoutTime;
    AprilTags                 *aprilTags;
    Drivetrain                *drivetrain;
};

class ChassisDriveToAprilTag : public frc2::CommandHelper<frc2::Command, ChassisDriveToAprilTag>
{
    public:

        explicit ChassisDriveToAprilTag(units::meters_per_second_t speed,
                                        units::meter_t             distanceOffsetX,
                                        units::meter_t             distanceOffsetY,
                                        units::degree_t            angleOffset,
                                        units::time::second_t      timeoutTime,
                                        AprilTags                 *aprilTags,
                                        Drivetrain                *drivetrain);

        explicit ChassisDriveToAprilTag(std::function<ChassDriveAprilTagParameters()> getParameters);

        void     Initialize()          override;
        void     Execute()             override;
        bool     IsFinished()          override;
        void     End(bool interrupted) override;

    private:

        units::meters_per_second_t        m_speed;                    // The speed of the chassis
        units::time::second_t             m_timeoutTime;              // The command time-out time
        units::meter_t                    m_distanceOffsetX;          // The distance offset in the X direction
        units::meter_t                    m_distanceOffsetY;          // The distance offset in the Y direction
        units::degree_t                   m_angleOffset;              // The angle offset
        AprilTags                        *m_aprilTags;                // The AprilTag subsystem
        Drivetrain                       *m_drivetrain;               // The drivetrain subsystem

        units::second_t                   m_startTime;                // The start of the drive time
        frc2::SwerveControllerCommand<4> *m_swerveControllerCommand;  // The swerve controller command
};
