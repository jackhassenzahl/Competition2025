#include "commands/GrabberPose.h"

/// @brief Method to set the grabber to a specific pose for placing or picking up a game piece.
/// @param armAngle The angle to set the arm.
/// @param elevatorHeight The height to set the elevator.
/// @param wristAngle The angle to set the wrist.
/// @param grabberWheelVelocity The velocity to set the grabber wheels.
/// @param arm The arm subsystem.
/// @param elevator The elevator subsystem.
/// @param grabber The grabber subsystem.
GrabberPose::GrabberPose(units::degree_t armAngle,   units::meter_t elevatorHeight,
                         units::degree_t wristAngle, double         grabberWheelVelocity,
                         Arm *arm, Elevator *elevator, Grabber *grabber) :
                         m_armAngle(armAngle), m_elevatorHeight(elevatorHeight),
                         m_wristAngle(wristAngle), m_wheeelVelocity(grabberWheelVelocity),
                         m_arm(arm), m_elevator(elevator), m_grabber(grabber)

{
    // Perform the following commands in parallel
    AddCommands(ArmSetAngle      (m_armAngle,                     m_arm),
                ElevatorSetHeight(m_elevatorHeight,               m_elevator),
                GrabberSetIntake (m_wristAngle, m_wheeelVelocity, m_grabber));
}
