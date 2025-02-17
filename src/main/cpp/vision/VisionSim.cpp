
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <photon/simulation/VisionSystemSim.h>

#include "vision/VisionSim.h"

#include "Constants.h"

using namespace physical::vision;

VisionSim::VisionSim( photon::PhotonCamera **camera )
{
    // Create array of camera sims
    camSims = new photon::PhotonCameraSim *[ kNumberOfCameras ];

    photon::SimCameraProperties cameraProp{};
    cameraProp.SetCalibError( 0.2, 0.08 );
    cameraProp.SetAvgLatency( 30_ms );
    cameraProp.SetLatencyStdDev( 5_ms );

    // Create each camera sim
    for( int i=0; i<kNumberOfCameras; ++i ) {
        camSims[i] = new photon::PhotonCameraSim{ camera[i], cameraProp };
        camSims[i]->EnableRawStream(false);
        camSims[i]->EnabledProcessedStream(false);
        visionSim.AddCamera( camSims[i], cameraInfo[i].robotToCamera );
    }

    // Load in the april tags
    frc::AprilTagFieldLayout tags = frc::AprilTagFieldLayout::LoadField( frc::AprilTagField::k2025ReefscapeWelded );
    visionSim.AddAprilTags( tags );

}

void VisionSim::Update( frc::Pose2d pose )
{
    visionSim.Update( pose );
}

