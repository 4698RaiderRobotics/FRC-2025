
#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include "command/ReefCommands.h"
#include "command/DriveToPose.h"

#include "arm/Arm.h"
#include "swerve/Drive.h"
#include "intake/Intake.h"
#include "elevator/Elevator.h"

#include "Constants.h"

using namespace physical;


ReefPlacingPoses ReefCommands::reefPoses = ReefPlacingPoses();

ReefPlacingPoses::ReefPlacingPoses() {
    const int Red_Reef_Tag_IDs[] = { 6, 7, 8, 9, 10, 11 };
    const int Blue_Reef_Tag_IDs[] = { 17, 18, 19, 20, 21, 22 };
    const units::inch_t pose_shift_out = 21_in;
    const units::inch_t pose_shift_lateral = 8_in;

    aprilTags = frc::AprilTagFieldLayout::LoadField( frc::AprilTagField::k2025ReefscapeWelded );

    // Placing pose is out from the april tag and to the left or right (looking at the AprilTag)
    frc::Transform2d leftShift{ pose_shift_out, -pose_shift_lateral, 180_deg };
    frc::Transform2d rightShift{ pose_shift_out, pose_shift_lateral, 180_deg };

    // Construct all the placing poses for the tags facing toward the driver
    for( int i=0; i<3; ++i ) {
        // Red Side Poses
        frc::Pose2d redTag2dPose = aprilTags.GetTagPose( Red_Reef_Tag_IDs[i] ).value().ToPose2d();
        redLeftReefPlacingPoses.push_back( redTag2dPose.TransformBy( leftShift ) );
        redRightReefPlacingPoses.push_back( redTag2dPose.TransformBy( rightShift ) );

        // Blue Side Poses
        frc::Pose2d blueTag2dPose = aprilTags.GetTagPose( Blue_Reef_Tag_IDs[i] ).value().ToPose2d();
        blueLeftReefPlacingPoses.push_back( blueTag2dPose.TransformBy( leftShift ) );
        blueRightReefPlacingPoses.push_back( blueTag2dPose.TransformBy( rightShift ) );
    }

    // Construct all the placing poses for the tags facing away from the driver
    // We swap what is left and right.
    for( int i=3; i<6; ++i ) {
        // Red Side Poses
        frc::Pose2d redTag2dPose = aprilTags.GetTagPose( Red_Reef_Tag_IDs[i] ).value().ToPose2d();
        redLeftReefPlacingPoses.push_back( redTag2dPose.TransformBy( rightShift ) );
        redRightReefPlacingPoses.push_back( redTag2dPose.TransformBy( leftShift ) );

        // Blue Side Poses
        frc::Pose2d blueTag2dPose = aprilTags.GetTagPose( Blue_Reef_Tag_IDs[i] ).value().ToPose2d();
        blueLeftReefPlacingPoses.push_back( blueTag2dPose.TransformBy( rightShift ) );
        blueRightReefPlacingPoses.push_back( blueTag2dPose.TransformBy( leftShift ) );
    }

}

frc2::CommandPtr ElevatorRaisePosition( Arm *arm )
{
    return frc2::cmd::Parallel(
        arm->ChangeElbowAngle( arm::kElbowRaiseAngle ),
        arm->ChangeWristPosition( ArmIO::WristHorizontal )
    );
}

frc2::CommandPtr ReefCommands::DriveToReefPose( Drive *d, bool onRightSide )
{
    return DriveToPose( d, [d, onRightSide] {
        frc::Pose2d currentPose = d->GetPose();

        return ReefCommands::reefPoses.GetClosest( currentPose, onRightSide );
    }
    ).WithName("DriveToReefPose");
}

frc2::CommandPtr ReefCommands::PlaceCoralL1( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        ElevatorRaisePosition( arm ),
        elevator->ChangeHeight( elevator::kHeightCoralL1 ),
        arm->ChangeElbowAngle( arm::kElbowCoralL1 ),
        intake->EjectCoral()
    ).WithName( "Place Coral in L1" );
}

frc::Pose2d ReefPlacingPoses::GetClosest( frc::Pose2d currentPose, bool onRightSide ) 
{
    if( frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ) {
        if( onRightSide ) {
            return currentPose.Nearest( redRightReefPlacingPoses );
        } else {
            return currentPose.Nearest( redLeftReefPlacingPoses );
        }
    } else {
        if( onRightSide ) {
            return currentPose.Nearest( blueRightReefPlacingPoses );
        } else {
            return currentPose.Nearest( blueLeftReefPlacingPoses );
        }
    }
}