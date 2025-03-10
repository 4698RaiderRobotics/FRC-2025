
#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "command/ReefCommands.h"
#include "command/IntakeCommands.h"
#include "command/DriveCommands.h"

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
    const units::inch_t pose_shift_out = 30_in;
    const units::inch_t pose_shift_lateral = 7.5_in;

    aprilTags = frc::AprilTagFieldLayout::LoadField( frc::AprilTagField::k2025ReefscapeWelded );

    // Placing pose is out from the april tag and to the left or right (looking at the AprilTag)
    frc::Transform2d algaeShift{ pose_shift_out, 0_in, 180_deg };
    frc::Transform2d leftShift{ pose_shift_out, -pose_shift_lateral, 180_deg };
    frc::Transform2d rightShift{ pose_shift_out, pose_shift_lateral, 180_deg };

    // Construct all the placing poses for the tags facing toward the driver
    for( int i=0; i<3; ++i ) {
        // Red Side Poses
        frc::Pose2d redTag2dPose = aprilTags.GetTagPose( Red_Reef_Tag_IDs[i] ).value().ToPose2d();
        redAlgaeRemovingPoses.push_back( redTag2dPose.TransformBy ( algaeShift ) );
        redLeftReefPlacingPoses.push_back( redTag2dPose.TransformBy( leftShift ) );
        redRightReefPlacingPoses.push_back( redTag2dPose.TransformBy( rightShift ) );

        // Blue Side Poses
        frc::Pose2d blueTag2dPose = aprilTags.GetTagPose( Blue_Reef_Tag_IDs[i] ).value().ToPose2d();
        blueAlgaeRemovingPoses.push_back( blueTag2dPose.TransformBy ( algaeShift ) );
        blueLeftReefPlacingPoses.push_back( blueTag2dPose.TransformBy( leftShift ) );
        blueRightReefPlacingPoses.push_back( blueTag2dPose.TransformBy( rightShift ) );
    }

    // Construct all the placing poses for the tags facing away from the driver
    // We swap what is left and right.
    for( int i=3; i<6; ++i ) {
        // Red Side Poses
        frc::Pose2d redTag2dPose = aprilTags.GetTagPose( Red_Reef_Tag_IDs[i] ).value().ToPose2d();
        redAlgaeRemovingPoses.push_back( redTag2dPose.TransformBy ( algaeShift ) );
        redLeftReefPlacingPoses.push_back( redTag2dPose.TransformBy( rightShift ) );
        redRightReefPlacingPoses.push_back( redTag2dPose.TransformBy( leftShift ) );

        // Blue Side Poses
        frc::Pose2d blueTag2dPose = aprilTags.GetTagPose( Blue_Reef_Tag_IDs[i] ).value().ToPose2d();
        blueAlgaeRemovingPoses.push_back( blueTag2dPose.TransformBy ( algaeShift ) );
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

frc2::CommandPtr ReefCommands::PlaceOnReef( 
    Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide, std::function<ReefPlacement ()> place_func )
{
    return frc2::cmd::Sequence(
        frc2::cmd::Either( 
            frc2::cmd::Sequence(
                DriveToReefPose( d, onRightSide ),
                frc2::cmd::Select<ReefPlacement>( 
                    [place_func] { return place_func(); }, 
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
    intake->EjectCoralL2_4( false ),      // Force Eject
    frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
                        DriveCommands::DriveDeltaPose( d, {-12_in, 0_in, 0_deg}, true ),
    frc2::cmd::RunOnce( [intake] { intake->Stop(); }),
                        IntakeCommands::RestPosition( arm, intake, elevator )
                    ),
                    // [intake] { return intake->isCenterBroken(); }
    [] {return false; }  // Always Eject
                )
            ),
            frc2::cmd::Print( "No Reef Level Selected!!"),
            [place_func] { return place_func() != ReefPlacement::NONE; }
        )
    ).WithName("PlaceOnReef");
}

frc2::CommandPtr ReefCommands::DriveToReefPose( Drive *d, bool onRightSide )
{
    return DriveCommands::DriveToPosePP( d, [d, onRightSide] {
        return ReefCommands::reefPoses.GetClosestReefPose( d->GetPose(), onRightSide );
    }
    ).WithName("DriveToReefPose");
}

frc2::CommandPtr ReefCommands::DriveToAlgaePose( Drive *d )
{
    return DriveCommands::DriveToPosePP( d, [d] {
        return ReefCommands::reefPoses.GetClosestAlgaePose( d->GetPose() );
    }
    ).WithName("DriveToAlgaePose");
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
        DriveCommands::DriveDeltaPose( d, {6_in, 0_in, 0_deg}, true ),
        // DriveCommands::DriveOpenLoop( d, {1_fps, 0_fps, 0_rpm}, true ).WithTimeout( 0.2_s),
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
            DriveCommands::DriveOpenLoop( d, {1_fps, 0_fps, 0_rpm}, true ).WithTimeout( 0.5_s)
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
            DriveCommands::DriveOpenLoop( d, {1.1_fps, 0_fps, 0_rpm}, true ).WithTimeout( 1_s)
        )
    ).WithName( "Place Coral in L4" );
}

frc2::CommandPtr ReefCommands::RemoveAlgae( Drive *d, Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence( 
        DriveToAlgaePose( d ),
        ElevatorRaisePosition( arm ),
        frc2::cmd::Either( 
            elevator->ChangeHeight( elevator::kHeightLowAlgae ),
            elevator->ChangeHeight( elevator::kHeightHighAlgae ),
            [d] { return ReefCommands::reefPoses.isAlgaeLow( d->GetPose() ); }
        ),
        arm->ChangeElbowAngle( arm::kElbowRemoveAlgae ),
        frc2::cmd::RunOnce( [intake] {intake->SpinOut();}, {intake} ),
        DriveCommands::DriveDeltaPose( d, {6_in, 0_in, 0_deg}, true ),
        frc2::cmd::Either( 
            elevator->ChangeHeight( elevator::kHeightLowAlgae + 4_in ),
            elevator->ChangeHeight( elevator::kHeightHighAlgae + 4_in ),
            [d] { return ReefCommands::reefPoses.isAlgaeLow( d->GetPose() ); }
        ),
        arm->ChangeElbowAngle( arm::kElbowRemoveAlgaeEnd ),
        DriveCommands::DriveDeltaPose( d, {-6_in, 0_in, 0_deg}, true )
    );
}

frc::Pose2d ReefPlacingPoses::GetClosestReefPose( frc::Pose2d currentPose, bool onRightSide ) 
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

frc::Pose2d ReefPlacingPoses::GetClosestAlgaePose( frc::Pose2d currentPose ) 
{
    if( frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ) {
        return currentPose.Nearest( redAlgaeRemovingPoses );
    } else {
        return currentPose.Nearest( blueAlgaeRemovingPoses );
    }
}

bool ReefPlacingPoses::isAlgaeLow( frc::Pose2d currentPose ) 
{
    frc::Pose2d closest_Pose;
    int matched_pose_idx = 0;

    if( frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ) {
        closest_Pose = currentPose.Nearest( redAlgaeRemovingPoses );
        for( size_t i=0; i<6; ++i ) {
            if( redAlgaeRemovingPoses[i] == closest_Pose ) {
                matched_pose_idx = i;
                break;
            }
        }
    } else {
        closest_Pose = currentPose.Nearest( blueAlgaeRemovingPoses );
        for( size_t i=0; i<6; ++i ) {
            if( blueAlgaeRemovingPoses[i] == closest_Pose ) {
                matched_pose_idx = i;
                break;
            }
        }
    }

    return ( matched_pose_idx % 2 == 0 );  // 0, 2, 4 are low
}

