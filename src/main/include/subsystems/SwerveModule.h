#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

class SwerveModule
{
    public:
        SwerveModule(int driveMotorCANid, int angleMotorCANid, int angleEncoderCANid);

        void SetModuleState(float speed, float angle);

    private:
        std::unique_ptr<ctre::phoenix6::hardware::TalonFX>  talonMotor;
        std::unique_ptr<ctre::phoenix6::hardware::CANcoder> encoder;
};
