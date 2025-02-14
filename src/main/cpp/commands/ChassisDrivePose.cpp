#include "commands/ChassisDrivePose.h"

#pragma region ChassisDrivePose
/// @brief  Command to drive the chassis to a specific pose.
/// @param speed The speed to move the chassis.
/// @param distanceX The distance to move the chassis in the X direction.
/// @param distanceY The distance to move the chassis in the Y direction.
/// @param angle The angle to move the chassis.
/// @param drivetrain The Drivetrain subsystem.
ChassisDrivePose::ChassisDrivePose(units::velocity::meters_per_second_t speed, units::meter_t distanceX, units::meter_t distanceY,
                                   units::angle::degree_t angle, units::time::second_t timeoutTime, Drivetrain *drivetrain) :
                                   m_speed(speed), m_distanceX(distanceX), m_distanceY(distanceY), m_angle(angle),
                                   m_timeoutTime(timeoutTime), m_drivetrain(drivetrain)
{
    // Set the command name
    SetName("ChassisDrivePose");

    // Declare subsystem dependencies
    AddRequirements(m_drivetrain);

    // Ensure the SwerveControllerCommand is set to nullptr
    m_swerveControllerCommand = nullptr;
}
#pragma endregion

#pragma region Initialize
/// @brief Called just before this Command runs.
void ChassisDrivePose::Initialize()
{
    try
    {
        // Set up config for trajectory
        frc::TrajectoryConfig trajectoryConfig(m_speed, PoseConstants::MaxAcceleration);

        // Add kinematics to ensure maximum speed is actually obeyed
        trajectoryConfig.SetKinematics(m_drivetrain->m_kinematics);

        // Ensure the new pose requires an X or Y move
        // Note: GenerateTrajectory will throw an exception if the distance X and Y are zero
        if (fabs(m_distanceX.value()) < 0.001 && fabs(m_distanceY.value()) < 0.001)
            m_distanceX = 0.01_m;

        // Get the robot starting pose
        auto startPose = m_drivetrain->GetPose();

        // Create the trajectory to follow
        frc::Pose2d endPose{startPose.X()                  + m_distanceX,
                            startPose.Y()                  + m_distanceY,
                            startPose.Rotation().Degrees() + m_angle};

        frc::SmartDashboard::PutNumber("DistanceX", m_distanceX.value());
        frc::SmartDashboard::PutNumber("DistanceY", m_distanceY.value());
        frc::SmartDashboard::PutNumber("Angle",     m_angle.value());

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
void ChassisDrivePose::Execute()
{
    // Execute the swerve controller command
    if (m_swerveControllerCommand)
        m_swerveControllerCommand->Execute();
}
#pragma endregion

#pragma region IsFinished
/// @brief Indicates if the command has completed.
/// @return True is the command has completed.
bool ChassisDrivePose::IsFinished()
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
void ChassisDrivePose::End(bool interrupted)
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
