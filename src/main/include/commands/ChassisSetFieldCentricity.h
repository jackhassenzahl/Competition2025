#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Drivetrain.h"

class ChassisSetFieldCentricity : public frc2::CommandHelper<frc2::Command, ChassisSetFieldCentricity>
{
    public:

        explicit ChassisSetFieldCentricity(bool fieldCentric, Drivetrain *drivetrain);

        void Initialize() override;
        bool IsFinished() override;
        
    private:

        bool        m_fieldCentric;  // The field centricity flag
        Drivetrain *m_drivetrain;    // Pointer to the chassis set field centricity class
};
