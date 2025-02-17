#pragma once

#include "VisionSim.h"

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <photon/simulation/VisionSystemSim.h>

class VisionSim {
public:
    VisionSim( photon::PhotonCamera **cameras );
    void Update( frc::Pose2d pose );

private:
    photon::VisionSystemSim visionSim{ "main" };
    photon::PhotonCameraSim **camSims;
};
