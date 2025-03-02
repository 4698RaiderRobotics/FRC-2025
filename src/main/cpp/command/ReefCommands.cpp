
#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

#include "command/ReefCommands.h"
#include "command/IntakeCommands.h"
#include "command/DriveCommands.h"
#include "command/DriveToPose.h"

#include "arm/Arm.h"
#include "swerve/Drive.h"
#include "intake/Intake.h"
#include "elevator/Elevator.h"

#include "Constants.h"

using namespace physical;

ReefPlacement ReefCommands::next_reef_place = ReefPlacement::NONE;

ReefPlacingPoses ReefCommands::reefPoses = ReefPlacingPoses();

ReefPlacingPoses::ReefPlacingPoses() {
    const int Red_Reef_Tag_IDs[] = { 6, 7, 8, 9, 10, 11 };
    const int Blue_Reef_Tag_IDs[] = { 17, 18, 19, 20, 21, 22 };
    const units::inch_t pose_shift_out = 21_in;
    const units::inch_t pose_shift_lateral = 7_in;

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
    return frc2::cmd::Sequence(
        arm->ChangeElbowAngle( arm::kElbowForwardRaiseAngle ),
        arm->ChangeWristPosition( ArmIO::WristHorizontal )
    );
}

frc2::CommandPtr ReefCommands::PlaceOnReef( Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide )
{
    return frc2::cmd::Sequence(
        frc2::cmd::Either( 
            frc2::cmd::Sequence(
                DriveToReefPose( d, onRightSide ),
                frc2::cmd::Select<ReefPlacement>( 
                    [] { return next_reef_place; }, 
                    std::pair{ ReefPlacement::NONE, frc2::cmd::Print( "No Reef Level Selected!!") },
                    std::pair{ ReefPlacement::PLACING_L1, PlaceCoralL1( d, arm, intake, elevator ) },
                    std::pair{ ReefPlacement::PLACING_L2, PlaceCoralL2( d, arm, intake, elevator ) },
                    std::pair{ ReefPlacement::PLACING_L3, PlaceCoralL3( d, arm, intake, elevator ) },
                    std::pair{ ReefPlacement::PLACING_L4, PlaceCoralL4( d, arm, intake, elevator ) }
                ),
                frc2::cmd::Either( 
                    // Still have the coral
                    frc2::cmd::None(),
                    // Ejected the coral
                    frc2::cmd::Sequence(
                        ReefCommands::SetReefPlacement(ReefPlacement::NONE),
                        DriveCommands::DriveDeltaPose( d, {-6_in, 0_in, 0_deg}, true ),
                        IntakeCommands::RestPosition( arm, intake, elevator )
                    ),
                    [intake] { return intake->isCenterBroken(); }
                )
            ),
            frc2::cmd::Print( "No Reef Level Selected!!"),
            [] { return next_reef_place != ReefPlacement::NONE; }
        )
    ).WithName("PlaceOnReef");
}

frc2::CommandPtr ReefCommands::DriveToReefPose( Drive *d, bool onRightSide )
{
    return DriveToPose( d, [d, onRightSide] {
        frc::Pose2d currentPose = d->GetPose();

        return ReefCommands::reefPoses.GetClosest( currentPose, onRightSide );
    },
    0.5 // Go at half speed
    ).WithName("DriveToReefPose");
}

frc2::CommandPtr ReefCommands::PlaceCoralL1( Drive *d, Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        arm->ChangeElbowAngle( arm::kElbowForwardRaiseAngle ),
        frc2::cmd::Parallel(
            elevator->ChangeHeight( elevator::kHeightCoralL1 ),
            arm->ChangeElbowAngle( arm::kElbowCoralL1 )
        ),
        arm->ChangeWristPosition( ArmIO::WristHorizontal ),
        DriveCommands::DriveDeltaPose( d, {3_in, 0_in, 0_deg}, true ),
        intake->EjectCoralL1()
    ).WithName( "Place Coral in L1" );
}

frc2::CommandPtr ReefCommands::PlaceCoralL2( Drive *d, Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        ElevatorRaisePosition( arm ),
        elevator->ChangeHeight( elevator::kHeightCoralL2 ),
        arm->ChangeWristPosition( ArmIO::WristVertical ),
        arm->ChangeElbowAngle( arm::kElbowCoralL2 ),
        frc2::cmd::Parallel(
            intake->EjectCoralL2_4( true ),
            DriveCommands::DriveDeltaPose( d, {3_in, 0_in, 0_deg}, true )
        )
    ).WithName( "Place Coral in L2" );
}

frc2::CommandPtr ReefCommands::PlaceCoralL3( Drive *d, Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        ElevatorRaisePosition( arm ),
        elevator->ChangeHeight( elevator::kHeightCoralL3 ),
        arm->ChangeWristPosition( ArmIO::WristVertical ),
        arm->ChangeElbowAngle( arm::kElbowCoralL3 ),
        frc2::cmd::Parallel(
            intake->EjectCoralL2_4( true ),
            DriveCommands::DriveOpenLoop( d, {1_fps, 0_fps, 0_rpm}, true ).WithTimeout( 1_s)
        )
    ).WithName( "Place Coral in L3" );
}

frc2::CommandPtr ReefCommands::PlaceCoralL4( Drive *d, Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        ElevatorRaisePosition( arm ),
        elevator->ChangeHeight( elevator::kHeightCoralL4 ),
        arm->ChangeWristPosition( ArmIO::WristVertical ),
        arm->ChangeElbowAngle( arm::kElbowCoralL4 ),
        frc2::cmd::Parallel(
            intake->EjectCoralL2_4( true ),
            DriveCommands::DriveOpenLoop( d, {1_fps, 0_fps, 0_rpm}, true ).WithTimeout( 1_s)
        )
    ).WithName( "Place Coral in L4" );
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

frc2::CommandPtr ReefCommands::SetReefPlacement( ReefPlacement p )
{
    return frc2::cmd::RunOnce( [p] { next_reef_place = p; LogReefPlacement( p ); } );
}

void ReefCommands::LogReefPlacement( ReefPlacement p )
{
    std::string logStr;
    
    switch( p ) {
    case ReefPlacement::NONE:
        logStr = "NONE";
        break;
    case ReefPlacement::PLACING_L1:
        logStr = "PLACING_L1";
        break;
    case ReefPlacement::PLACING_L2:
        logStr = "PLACING_L2";
        break;
    case ReefPlacement::PLACING_L3:
        logStr = "PLACING_L3";
        break;
    case ReefPlacement::PLACING_L4:
        logStr = "PLACING_L4";
        break;
    }

    DataLogger::Log( "ReefCommands/Reef Place", logStr );
}
