#include "commands/ChassisDriveToWall.h"

#pragma region
/// @brief Command to drive the chassis to an AprilTag.
/// @param speed The speed to move the chassis.
/// @param timeoutTime The timeout time for the move.
/// @param aprilTags The AprilTag subsystem.
/// @param drivetrain The Drivetrain subsystem.
ChassisDriveToWall::ChassisDriveToWall(units::meters_per_second_t speed, units::meter_t distanceToWall,
                                       units::time::second_t timeoutTime, Drivetrain *drivetrain) :
                                       m_speed(speed), m_distanceToWall(distanceToWall),
                                       m_timeoutTime(timeoutTime), m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDriveToWall");

    // Declare subsystem dependencies
    AddRequirements(m_drivetrain);

    // Ensure the SwerveControllerCommand is set to nullptr
    m_swerveControllerCommand = nullptr;
}
#pragma endregion

#pragma region Initialize
/// @brief Called just before this Command runs the first time.
void ChassisDriveToWall::Initialize()
{
    // Set up config for trajectory
    frc::TrajectoryConfig trajectoryConfig(m_speed, PoseConstants::MaxAcceleration);

    // Add kinematics to ensure maximum speed is actually obeyed
    trajectoryConfig.SetKinematics(m_drivetrain->m_kinematics);

    // Get the present destince to the wall
    auto distanceToWall = m_drivetrain->GetDistance();

    // Determine how far to move towards the wall
    auto distance = distanceToWall - m_distanceToWall;

    // Get the present robot pose
    auto startingPose = m_drivetrain->GetPose();

    // Calculate the end 
    frc::Pose2d endPose = frc::Pose2d{startingPose.X() + distance, startingPose.Y(), startingPose.Rotation()};
    
    // Create the trajectory to follow
    auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(startingPose, {}, endPose, trajectoryConfig);

    // Create a profile PID controller
    frc::ProfiledPIDController<units::radians> profiledPIDController{PoseConstants::PProfileController, 0, 0,
                                                                     PoseConstants::ThetaControllerConstraints};

    // enable continuous input for the profile PID controller
    profiledPIDController.EnableContinuousInput(units::radian_t{-std::numbers::pi}, units::radian_t{std::numbers::pi});

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

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain->ResetOdometry(trajectory.InitialPose());

    // Initialize the swerve controller command
    m_swerveControllerCommand->Initialize();

    // Get the start time
    m_startTime = frc::GetTime();
}
#pragma endregion

#pragma region Execute
/// @brief Called repeatedly when this Command is scheduled to run.
void ChassisDriveToWall::Execute()
{
    // Execute the swerve controller command
    if (m_swerveControllerCommand)
        m_swerveControllerCommand->Execute();
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool ChassisDriveToWall::IsFinished()
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
void ChassisDriveToWall::End(bool interrupted)
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