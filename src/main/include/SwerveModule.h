#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

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

        WheelVector m_wheelVector;

        std::unique_ptr<ctre::phoenix6::hardware::TalonFX>  talonMotor;
        std::unique_ptr<ctre::phoenix6::hardware::CANcoder> encoder;

        // Private methods
        void   OptimizeWheelAngle(WheelVector targetWheelVector, WheelVector *wheelVector);
        double ConvertAngleToTargetRange(WheelVector wheelVector, WheelVector targetWheelVector);
};
