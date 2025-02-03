#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include "Constants.h"

class Arm : public frc2::SubsystemBase
{
    public:

        Arm();

        void SetHeight(units::length::meter_t position);

    private:

        void ConfigureArmMotor(int driveMotorCanId);

        ctre::phoenix6::hardware::TalonFX           *m_armMotor;
        ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicVoltage{0_tr};
};
