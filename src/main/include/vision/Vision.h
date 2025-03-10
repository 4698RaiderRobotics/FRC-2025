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
    std::string camera_name;
    std::optional<frc::Pose2d> visionPose;
    std::vector<frc::Pose3d> trackedTags;
    units::meter_t tagDistance{0_m};
    units::meter_t deltaDistance{0_m};
    double std_dev{0.0};
    units::second_t timestamp{0_s};
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

    bool hasTrackedTags() { return tracked_tags >= 10;}
    
private:
    void ProcessCamera( int cameraNumber );

    photon::PhotonCamera **cameras;
    photon::PhotonPoseEstimator **estimators;

    std::unique_ptr<VisionSim> sim;
    frc::SwerveDrivePoseEstimator<4> *odometry;
    frc::AprilTagFieldLayout aprilTags;
    Metrics metrics;
    size_t tracked_tags{0};
};
