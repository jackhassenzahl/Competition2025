#include "Constants.h"

namespace AutoConstants
{
    const frc::TrapezoidProfile<units::radians>::Constraints ThetaControllerConstraints{MaxAngularSpeed, MaxAngularAcceleration};
}
