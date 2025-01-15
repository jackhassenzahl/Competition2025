#pragma once

#include <frc2/command/SubsystemBase.h>

class AprilTags : public frc2::SubsystemBase
{
    public:

        explicit AprilTags();

        void     Periodic() override;

    private:

};
