#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include "Constants.h"

class Elevator : public frc2::SubsystemBase
{
    public:

        Elevator();

        void SetHeight(units::length::meter_t position);

    private:

        void ConfigureElevatorMotor(int driveMotorCanId);

        ctre::phoenix6::hardware::TalonFX           *m_elevatorMotor;
        ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicVoltage{0_tr};
};
