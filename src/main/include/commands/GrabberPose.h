#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>

#include <frc2/command/CommandHelper.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "subsystems/Arm.h"
#include "subsystems/Elevator.h"
#include "subsystems/Grabber.h"

#include "commands/ArmSetAngle.h"
#include "commands/ElevatorSetHeight.h"
#include "commands/GrabberSetIntake.h"

#include "Constants.h"

class GrabberPose : public frc2::CommandHelper<frc2::ParallelCommandGroup, GrabberPose>
{
    public:

        GrabberPose(units::degree_t armAngle,
                    units::meter_t  elevatorHeight,
                    units::degree_t wristAngle,
                    double          grabberWheelVelocity,
                    Arm *arm, Elevator *elevator, Grabber *grabber);

    private:

        units::degree_t m_armAngle;
        units::meter_t  m_elevatorHeight;
        units::degree_t m_wristAngle;
        double          m_wheeelVelocity;

        Arm             *m_arm;
        Elevator        *m_elevator;
        Grabber         *m_grabber;
};
