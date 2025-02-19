#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Climb.h"


class ClimbSetAngleOffset : public frc2::CommandHelper<frc2::Command, ClimbSetAngleOffset> 
{
  public:
  
      ClimbSetAngleOffset(units::angle::degree_t climbOffset, Climb *climb);
      
      void Execute() override;

      bool IsFinished() override;

    private:
        units::angle::degree_t m_climbOffset;
        Climb* m_climb;
};
