#pragma once

#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include "Constants.h"

class Climb : public frc2::SubsystemBase
{
    public:

        Climb();

        void SetVoltage(units::volt_t voltage);

    private:

        void ConfigureClimbMotor(int motorCanId);

        ctre::phoenix6::hardware::TalonFX *m_climbMotor;

        frc::DigitalInput                  m_climbLimit  {ClimbConstants::ClimbLimitSwtich};
        frc::DigitalInput                  m_captureLimit{ClimbConstants::CaptureLimitSwitch};
};
