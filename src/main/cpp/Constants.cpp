#include "subsystems/Drivetrain.h"

#include <constants/xBoxConstants.h>
#include <constants/Extreme3DConstants.h>
#include <constants/ControllerConstants.h>
#include <constants/ControlPanelConstants.h>
#include <constants/CanConstants.h>

// #include "Constants.h"

namespace ChassisPoseConstants
{
    const frc::TrapezoidProfile<units::radians>::Constraints ThetaControllerConstraints{ChassisPoseConstants::MaxAngularSpeed,
                                                                                        ChassisPoseConstants::MaxAngularAcceleration};
}
