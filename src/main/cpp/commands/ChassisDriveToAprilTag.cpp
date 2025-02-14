#include "commands/ChassisDriveToAprilTag.h"

#pragma region
/// @brief Command to drive the chassis to an AprilTag.
/// @param speed The speed to move the chassis.
/// @param timeoutTime The timeout time for the move.
/// @param aprilTags The AprilTag subsystem.
/// @param drivetrain The Drivetrain subsystem.
ChassisDriveToAprilTag::ChassisDriveToAprilTag(units::meters_per_second_t speed,
                                               units::meter_t             distanceOffsetX,
                                               units::meter_t             distanceOffsetY,
                                               units::degree_t            angleOffset,
                                               units::time::second_t      timeoutTime,
                                               AprilTags                 *aprilTags,
                                               Drivetrain                *drivetrain) :
                                               m_speed(speed),                     m_distanceOffsetX(distanceOffsetX),
                                               m_distanceOffsetY(distanceOffsetY), m_angleOffset(angleOffset),                                        
                                               m_timeoutTime(timeoutTime),         m_aprilTags(aprilTags),
                                               m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDriveToAprilTag");

    // Declare subsystem dependencies
    AddRequirements(m_drivetrain);

    // Ensure the SwerveControllerCommand is set to nullptr
    m_swerveControllerCommand = nullptr;
}
#pragma endregion

#pragma region Initialize
/// @brief Called just before this Command runs.
/// @details Initializes the command.
/// Swerve module order for kinematics calculations
///
/// Apriltag Coordinates:
///
///    Z - The distance from the camera to the tag.
///    X - The distance from the camera to the tag in the X direction.
///    Y - The distance from the camera to the tag in the Y direction.
///
///             Camera           Translation2d Coordinates
///
///               |                         ^ X
///               |                         |
///         ------+------> X       Y <------+-------
///               |                         |
///               |                         |
///               V Y
///
/// Translation:  Pose X ->  Z
///               Pose Y -> -X
///               Pose A -> -Rotation Y

void ChassisDriveToAprilTag::Initialize()
{
    try
    {
        auto aprilTagInformation = m_aprilTags->GetClosestTag();

        // Determine if an AprilTag was found
        if (aprilTagInformation.Found == true)
        {
            frc::SmartDashboard::PutNumber("Distance X", aprilTagInformation.X);
            frc::SmartDashboard::PutNumber("Distance Y", aprilTagInformation.Y);
            frc::SmartDashboard::PutNumber("Distance Z", aprilTagInformation.Z);
            frc::SmartDashboard::PutNumber("Rotation X", aprilTagInformation.rotationX);
            frc::SmartDashboard::PutNumber("Rotation Y", aprilTagInformation.rotationY);
            frc::SmartDashboard::PutNumber("Rotation Z", aprilTagInformation.rotationZ);
        }
        else  // If no AprilTag is found, end the command
        {
            // End the command
            End(true);
        }

        // Set up config for trajectory
        frc::TrajectoryConfig trajectoryConfig(m_speed, PoseConstants::MaxAcceleration);

        // Add kinematics to ensure maximum speed is actually obeyed
        trajectoryConfig.SetKinematics(m_drivetrain->m_kinematics);

        // Ensure the new pose requires an X or Y move
        // Note: GenerateTrajectory will throw an exception if the distance X and Y are zero
        if (fabs(aprilTagInformation.X) < 0.001 && fabs(aprilTagInformation.Y) < 0.001)
            aprilTagInformation.X = 0.01;

        // Get the robot starting pose
        auto startPose = m_drivetrain->GetPose();


        // Offset the position based on the specified distances and angle
        auto distanceX   =  (units::meter_t)          aprilTagInformation.Z         - m_distanceOffsetX;
        auto distanceY   =  (units::meter_t)         -aprilTagInformation.X         + m_distanceOffsetY; 
        auto angleOffset =  (units::angle::radian_t) -aprilTagInformation.rotationY + m_angleOffset;

        // Create the trajectory to follow
        frc::Pose2d endPose{startPose.X()                  + distanceX,
                            startPose.Y()                  + distanceY,
                            startPose.Rotation().Degrees() + angleOffset};

        frc::SmartDashboard::PutNumber("DistanceX",  distanceX.value());
        frc::SmartDashboard::PutNumber("DistanceY",  distanceY.value());
        frc::SmartDashboard::PutNumber("Angle",      units::angle::degree_t(angleOffset).value());

        frc::SmartDashboard::PutNumber("StartX", startPose.X().value());
        frc::SmartDashboard::PutNumber("StartY", startPose.Y().value());
        frc::SmartDashboard::PutNumber("StartA", startPose.Rotation().Degrees().value());

        frc::SmartDashboard::PutNumber("EndX", endPose.X().value());
        frc::SmartDashboard::PutNumber("EndY", endPose.Y().value());
        frc::SmartDashboard::PutNumber("EndA", endPose.Rotation().Degrees().value());

        // Create the trajectory to follow
        auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(startPose, {}, endPose, trajectoryConfig);

        // Create a profile PID controller
        frc::ProfiledPIDController<units::radians> profiledPIDController{PoseConstants::PProfileController, 0, 0,
                                                                         PoseConstants::ThetaControllerConstraints};

        // enable continuous input for the profile PID controller
        profiledPIDController.EnableContinuousInput(units::radian_t{-std::numbers::pi}, units::radian_t{std::numbers::pi});

        // Create the swerve controller command
        m_swerveControllerCommand = new frc2::SwerveControllerCommand<4>(
            trajectory,
            [this]() { return m_drivetrain->GetPose(); },
            m_drivetrain->m_kinematics,
            frc::PIDController(PoseConstants::PXController, 0, 0),
            frc::PIDController(PoseConstants::PYController, 0, 0),
            profiledPIDController,
            [this](auto moduleStates) { m_drivetrain->SetModuleStates(moduleStates); },
            {m_drivetrain}
        );

        // Set odometry to the starting pose of the trajectory.
        m_drivetrain->ResetOdometry(trajectory.InitialPose());

        // Initialize the swerve controller command
        m_swerveControllerCommand->Initialize();

        // Get the start time
        m_startTime = frc::GetTime();
    }
    catch(const std::exception& exception)
    {
        frc::SmartDashboard::PutString("Debug", exception.what());
    }
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisDriveToAprilTag::Execute()
{
    // Execute the swerve controller command
    if (m_swerveControllerCommand)
        m_swerveControllerCommand->Execute();
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool ChassisDriveToAprilTag::IsFinished()
{
    // Determine if the time-out time has expired
    if (frc::GetTime() - m_startTime > m_timeoutTime)
        return true;

    // Determine if the swerve controller command is finished
    return m_swerveControllerCommand && m_swerveControllerCommand->IsFinished();
}
#pragma endregion

#pragma region End
/// @brief Called once after isFinished returns true.
/// @param interrupted Indicated that the command was interrupted.
void ChassisDriveToAprilTag::End(bool interrupted)
{
    // If the swerve controller command is not nullptr, end the command
    if (m_swerveControllerCommand)
    {
        // End the swerve controller command
        m_swerveControllerCommand->End(interrupted);

        // Delete the swerve controller command and nullify the pointer
        delete m_swerveControllerCommand;
        m_swerveControllerCommand = nullptr;
    }

    // Stop the move
    m_drivetrain->Drive(0_mps, 0_mps, 0_rad_per_s);
}
#pragma endregion