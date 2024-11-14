#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/PWMVictorSPX.h>

class Drivetrain : public frc2::SubsystemBase
{
public:

    Drivetrain();

    void Periodic()            override;
    void SimulationPeriodic() override;

private:

    frc::MecanumDrive m_mecanumDrive{m_motorController1, m_motorController2, m_motorController3, m_motorController4};
    frc::PWMVictorSPX m_motorController4{3};
    frc::PWMVictorSPX m_motorController3{2};
    frc::PWMVictorSPX m_motorController2{1};
    frc::PWMVictorSPX m_motorController1{0};
};
