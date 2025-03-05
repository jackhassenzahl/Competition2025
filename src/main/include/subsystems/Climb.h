#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include <frc/DigitalInput.h>
#include <frc2/command/SubsystemBase.h>

#include <constants/xBoxConstants.h>
#include <constants/Extreme3DConstants.h>
#include <constants/ControllerConstants.h>
#include <constants/ControlPanelConstants.h>
#include <constants/CanConstants.h>

// #include "Constants.h"

#pragma region ClimbConstants
namespace ClimbConstants
{
    constexpr auto ClimbVoltage       = 12_V;

    constexpr auto ClimbLimitSwtich   = 0;
    constexpr auto CaptureLimitSwitch = 1;
}
#pragma endregion

class Climb : public frc2::SubsystemBase
{
    public:

        explicit Climb();

        void     SetVoltage(units::volt_t voltage);

    private:

        void ConfigureClimbMotor(int motorCanId);

        ctre::phoenix6::hardware::TalonFX *m_climbMotor;

        frc::DigitalInput                  m_climbLimit  {ClimbConstants::ClimbLimitSwtich};
        frc::DigitalInput                  m_captureLimit{ClimbConstants::CaptureLimitSwitch};
};
