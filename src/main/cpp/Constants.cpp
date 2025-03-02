#include "subsystems/Drivetrain.h"

#include "Constants.h"

namespace ChassisPoseConstants
{
    const frc::TrapezoidProfile<units::radians>::Constraints ThetaControllerConstraints{ChassisPoseConstants::MaxAngularSpeed,
                                                                                        ChassisPoseConstants::MaxAngularAcceleration};
}
