#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Elevator.h"

class ElevatorSetHeight : public frc2::CommandHelper<frc2::Command, ElevatorSetHeight>
{
    public:

        explicit ElevatorSetHeight(units::length::meter_t height, Elevator *elevator);

        void     Execute()    override;
        bool     IsFinished() override;

    private:

        units::length::meter_t m_height;
        Elevator              *m_elevator;
};
