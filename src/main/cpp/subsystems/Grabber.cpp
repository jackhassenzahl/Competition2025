#include "subsystems/Grabber.h"


Grabber::Grabber() :
            m_grabberMotor(CanConstants::GrabberMotorCanId, rev::spark::SparkMax::MotorType::kBrushless),
            m_grabberEncoder(m_wristMotor.GetEncoder()),
            m_grabberTurnClosedLoopController(m_wristMotor.GetClosedLoopController()),
           
            m_wristMotor(CanConstants::WristMotorCanId, rev::spark::SparkMax::MotorType::kBrushless),
            m_wristEncoder(m_wristMotor.GetEncoder()),
            m_wristTurnClosedLoopController(m_wristMotor.GetClosedLoopController())
{
    ConfigGrabberMotor();
    ConfigGrabberMotor();
}


void Grabber::ConfigGrabberMotor()
{
    // Configure the angle motor
    static rev::spark::SparkMaxConfig sparkMaxConfig{};


    sparkMaxConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(GrabberConstants::GrabberMaxAmperage);
    sparkMaxConfig.encoder
        //.Inverted(true)
        .PositionConversionFactor(2.0 * std::numbers::pi)
        .VelocityConversionFactor(1);
    sparkMaxConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(GrabberConstants::GrabberP, GrabberConstants::GrabberI, GrabberConstants::GrabberD)
        //.OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true);
        //.PositionWrappingInputRange(0, 2 * std::numbers::pi);  TODO: Try these settings


    // Write the configuration to the motor controller
    m_grabberMotor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}


void Grabber::ConfigWristMotor()
{
    // Configure the angle motor
    static rev::spark::SparkMaxConfig sparkMaxConfig{};


    sparkMaxConfig
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake)
        .SmartCurrentLimit(GrabberConstants::WristMaxAmperage);
    sparkMaxConfig.encoder
        //.Inverted(true)
        .PositionConversionFactor(GrabberConstants::WristRadiansToMotorRevolutions)
        .VelocityConversionFactor(GrabberConstants::WristRadiansToMotorRevolutions / 60.0);
    sparkMaxConfig.closedLoop
        .SetFeedbackSensor(rev::spark::ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder)
        .Pid(GrabberConstants::WristP, GrabberConstants::WristI, GrabberConstants::WristD)
        //.OutputRange(-1, 1)
        // Enable PID wrap around for the turning motor. This will allow the
        // PID controller to go through 0 to get to the setpoint i.e. going
        // from 350 degrees to 10 degrees will go through 0 rather than the
        // other direction which is a longer route.
        .PositionWrappingEnabled(true);
        //.PositionWrappingInputRange(0, 2 * std::numbers::pi);  TODO: Try these settings


    // Write the configuration to the motor controller
    m_wristMotor.Configure(sparkMaxConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}


/// @brief
/// @param velocity
void Grabber::SetGrabberWheelsVelocity(double velocity)
{
    if (fabs(velocity) > GrabberConstants::GrabberMaxRevolutionsPerMinute)
    {
        return;
    }


    m_wristTurnClosedLoopController
        .SetReference(velocity, rev::spark::SparkMax::ControlType::kVelocity);
}


void Grabber::SetWristAngle(units::angle::degree_t position)
{
    // This is in radians but conversion sucks:
    double positionRadian = (position.value() * std::numbers::pi) / 180.0;
   
    m_wristTurnClosedLoopController
        .SetReference(positionRadian, rev::spark::SparkMax::ControlType::kPosition);
}






