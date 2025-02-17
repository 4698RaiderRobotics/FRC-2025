
#include <frc/RobotBase.h>
#include <frc/DriverStation.h>

#include "util/DataLogger.h"
#include "Constants.h"

#include "vision/Vision.h"

using namespace physical::vision;

Vision::Vision( frc::SwerveDrivePoseEstimator<4> *odom ) : odometry{odom}
{
    SetName( "Vision" );

    // Create array of camera pointers and each camera.
    cameras = new photon::PhotonCamera *[ kNumberOfCameras ];
    for( int i=0; i<kNumberOfCameras; ++ i ) {
        cameras[i] = new photon::PhotonCamera{ cameraInfo[i].name };
        VisionResultData vizResult;
        metrics.visionResult.push_back( vizResult );
    }

    aprilTags = frc::AprilTagFieldLayout::LoadField( frc::AprilTagField::k2025ReefscapeWelded );

    // Create array of pose estimators and each estimator.
    estimators = new photon::PhotonPoseEstimator *[ kNumberOfCameras ];
    for( int i=0; i<kNumberOfCameras; ++ i ) {
       estimators[i] = new photon::PhotonPoseEstimator{ aprilTags, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, cameraInfo[i].robotToCamera };
    }

    if( frc::RobotBase::IsSimulation() ) {
        sim = std::unique_ptr<VisionSim> (new VisionSim( cameras ));
    }
}

void Vision::Periodic()
{
    if( sim ) {
        sim->Update( odometry->GetEstimatedPosition() );
    }

    for( int i=0; i<kNumberOfCameras; ++i ) {
        ProcessCamera( i );
    }

    metrics.Log( "Vision" );
}

void Vision::ProcessCamera( int camNumber )
{
    photon::PhotonCamera *cam = cameras[camNumber];
    photon::PhotonPoseEstimator *estimator = estimators[camNumber];

    VisionResultData resultData;

    auto result = cam->GetLatestResult();

    if( result.HasTargets() && result.GetBestTarget().GetPoseAmbiguity() < 0.2 ) {

        auto pose = estimator->Update( result );
        if( pose.has_value() ) {

            resultData.visionPose = pose->estimatedPose.ToPose2d();
            resultData.timestamp = pose->timestamp;

            resultData.tagDistance = result.GetBestTarget().GetBestCameraToTarget().Translation().Norm();
            double dist_to_tag = resultData.tagDistance.value();
            resultData.std_dev = (0.792*dist_to_tag - 2.125)*dist_to_tag + 1.833;
            if(dist_to_tag < 1.3) {
                 resultData.std_dev = 0.4;
            }

                // Linear distance between the current robot pose and the vision pose
            resultData.deltaDistance = odometry->GetEstimatedPosition().RelativeTo(resultData.visionPose.value()).Translation().Norm();

                // Get all the Apriltags that are used by this result
            for( size_t i=0; i<pose->targetsUsed.size(); ++i ) {
                int id = pose->targetsUsed[i].GetFiducialId();
                if( id > 0 && aprilTags.GetTagPose(id) ) {
                    resultData.trackedTags.push_back( aprilTags.GetTagPose(id).value() );
                }
            }

            if( frc::DriverStation::IsDisabled() ) {
                    // Always use poses when disabled
                resultData.addToOdometry = true;
            } else if( resultData.deltaDistance < 1_m ) {
                // Use poses that are close to the robot while Enabled
                resultData.addToOdometry = true;
            }
        }
    }

    if( resultData.addToOdometry ) {
        odometry->AddVisionMeasurement( resultData.visionPose.value(), 
                                        resultData.timestamp, 
                                        {resultData.std_dev, resultData.std_dev, resultData.std_dev} );
    }

    metrics.visionResult[camNumber] = resultData;
}

void Vision::Metrics::Log( const std::string &key ) 
{
    for( size_t i=0; i<visionResult.size(); ++i ) {
        std::string subkey = fmt::format( "{}/Camera{}/", key, i );
        VisionResultData &vizData = visionResult[i];

        DataLogger::Log( subkey + "visionPose", vizData.visionPose );
        DataLogger::Log( subkey + "trackedTags", vizData.trackedTags );
        DataLogger::Log( subkey + "tagDistance", vizData.tagDistance );
        DataLogger::Log( subkey + "deltaDistance", vizData.deltaDistance );
        DataLogger::Log( subkey + "std_dev", vizData.std_dev );
        DataLogger::Log( subkey + "timestamp", vizData.timestamp );
        DataLogger::Log( subkey + "addToOdometry", vizData.addToOdometry );
    }
}
