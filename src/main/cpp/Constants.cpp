#include "Constants.h"

namespace ChassisPoseConstants
{
    const frc::TrapezoidProfile<units::radians>::Constraints ThetaControllerConstraints{MaxAngularSpeed, MaxAngularAcceleration};
}
