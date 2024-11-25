#pragma once

#include "AHRS.h"

#include <rev/CANSparkMax.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

struct WheelVector
{
    double Drive = 0.0;
    double Angle = 0.0;
};

class SwerveModule
{
    public:

        SwerveModule(int driveMotorCANid, int angleMotorCANid, int angleEncoderCANid);

        void SetState(WheelVector wheelVector);

        void GetWheelVector(WheelVector *wheelVector);

    private:

        // Private methods
        void   ConfigureDriveMotor(int driveMotorCANid);
        void   ConfigureAngleMotor(int angleMotorCANid, int angleEncoderCANid);

        void   OptimizeWheelAngle(WheelVector targetWheelVector, WheelVector *wheelVector);
        double ConvertAngleToTargetRange(WheelVector wheelVector);

        WheelVector                         m_wheelVector;

        ctre::phoenix6::hardware::TalonFX  *m_driveMotor;

        rev::CANSparkMax                   *m_angleMotor;
        rev::SparkRelativeEncoder          *m_angleEncoder;
        ctre::phoenix6::hardware::CANcoder *m_angleAbsoluteEncoder;
        rev::SparkPIDController            *m_pidController;
};
