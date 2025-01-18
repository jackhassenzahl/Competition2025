#pragma once

#include <cstdio>
#include <span>
#include <sstream>
#include <iostream>
#include <string>
#include <thread>

#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cameraserver/CameraServer.h>

#include <frc/TimedRobot.h>
#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/geometry/Transform3d.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class AprilTags : public frc2::SubsystemBase
{
    public:

        explicit AprilTags();

        void     Periodic() override;

    private:

};
