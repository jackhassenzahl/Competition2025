#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include "Constants.h"

class Climb : public frc2::SubsystemBase
{
    public:

        Climb();

        void SetAngle(units::angle::degree_t angle);

    private:

        void ConfigureClimbMotor(int motorCanId);

        ctre::phoenix6::hardware::TalonFX           *m_climbMotor;
        ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicVoltage{0_tr};
};