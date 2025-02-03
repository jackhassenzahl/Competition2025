#include "Constants.h"

namespace PoseConstants
{
    const frc::TrapezoidProfile<units::radians>::Constraints ThetaControllerConstraints{MaxAngularSpeed, MaxAngularAcceleration};
}
