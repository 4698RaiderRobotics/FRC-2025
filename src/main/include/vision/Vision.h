#pragma once

#include <string>
#include <vector>
#include <optional>

#include <frc2/command/SubsystemBase.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

#include <photon/PhotonPoseEstimator.h>

#include "VisionSim.h"

#include "util/Utility.h"

struct VisionResultData {
    std::optional<frc::Pose2d> visionPose;
    std::vector<frc::Pose3d> trackedTags;
    units::meter_t tagDistance;
    units::meter_t deltaDistance;
    double std_dev;
    units::second_t timestamp;
    bool addToOdometry{false};
};

class Vision : public frc2::SubsystemBase {
public:
    struct Metrics {
        std::vector<VisionResultData> visionResult;

        void Log( const std::string &key );
    };

    Vision( frc::SwerveDrivePoseEstimator<4> *odometry );
    void Periodic() override;
    
private:
    void ProcessCamera( int cameraNumber );

    photon::PhotonCamera **cameras;
    photon::PhotonPoseEstimator **estimators;

    std::unique_ptr<VisionSim> sim;
    frc::SwerveDrivePoseEstimator<4> *odometry;
    frc::AprilTagFieldLayout aprilTags;
    Metrics metrics;
};
