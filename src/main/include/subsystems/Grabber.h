#pragma once


#include <frc2/command/SubsystemBase.h>


#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>


#include "Constants.h"


class Grabber : public frc2::SubsystemBase
{
  public:
    Grabber();


    void SetGrabberWheelsVelocity(double velocity);


    void SetWristAngle(units::angle::degree_t position);


  private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.


    rev::spark::SparkMax m_grabberMotor;
    rev::spark::SparkClosedLoopController m_grabberTurnClosedLoopController;
    rev::spark::SparkRelativeEncoder      m_grabberEncoder;
   
    rev::spark::SparkMax m_wristMotor;
    rev::spark::SparkClosedLoopController m_wristTurnClosedLoopController;
    rev::spark::SparkRelativeEncoder      m_wristEncoder;


    void ConfigGrabberMotor();
    void ConfigWristMotor();
};
