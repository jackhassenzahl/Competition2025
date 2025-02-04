#pragma once

#include <cameraserver/CameraServer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <networktables/IntegerArrayTopic.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/apriltag/AprilTagDetection.h>
#include <frc/apriltag/AprilTagDetector.h>
#include <frc/apriltag/AprilTagPoseEstimator.h>
#include <frc/geometry/Transform3d.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/TimedRobot.h>

#include "Constants.h"

struct AprilTagInformation
{
    bool   Found;
    int    Identification;
    double X;
    double Y;
    double Z;
    double rotationX;
    double rotationY;
    double rotationZ;
};

class AprilTags : public frc2::SubsystemBase
{
    public:

        explicit            AprilTags();

        void                Periodic() override;

        bool                GetTag(int id, AprilTagInformation &aprilTagInformation);

        AprilTagInformation GetClosestTag();

    private:

        nt::NetworkTableInstance          m_instance;
        std::shared_ptr<nt::NetworkTable> m_aprilTagsTable;
        nt::IntegerArrayTopic             m_aprilTagsIntegerArrayTopic;
        nt::IntegerArraySubscriber        m_aprilTagsIntegerArraySubscriber;

        std::vector<AprilTagInformation>  m_detectedAprilTags;
};
