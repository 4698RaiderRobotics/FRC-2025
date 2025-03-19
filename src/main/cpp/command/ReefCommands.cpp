
#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "command/ReefCommands.h"
#include "command/IntakeCommands.h"
#include "command/DriveCommands.h"
#include "command/DriveToPose.h"
#include "command/ControllerIO.h"

#include "arm/Arm.h"
#include "swerve/Drive.h"
#include "intake/Intake.h"
#include "elevator/Elevator.h"
#include "climber/Climber.h"

#include "Constants.h"

using namespace physical;

ReefPlacingPoses ReefCommands::reefPoses = ReefPlacingPoses();

ReefPlacingPoses::ReefPlacingPoses() 
{
    const int Red_Reef_Tag_IDs[] = { 6, 7, 8, 9, 10, 11 };
    const int Blue_Reef_Tag_IDs[] = { 17, 18, 19, 20, 21, 22 };

    aprilTags = frc::AprilTagFieldLayout::LoadField( frc::AprilTagField::k2025ReefscapeWelded );

    // Placing pose is out from the april tag and to the left or right (looking at the AprilTag)
    frc::Transform2d algaeShift{ reef::algae_remove_pose_shift_out, 0_in, 180_deg };
    frc::Transform2d leftShift{ reef::pose_shift_out, -reef::pose_shift_lateral, 180_deg };
    frc::Transform2d rightShift{ reef::pose_shift_out, reef::pose_shift_lateral, 180_deg };

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

    // OutputPathPlannerJSON();
}

void ReefPlacingPoses::OutputPathPlannerJSON() 
{
    std::vector<frc::Pose2d> PP_poses;

    const char *PP_names[] = {
        "FrontProcessorLeft",  "FrontProcessorRight",  
        "FrontCenterLeft",  "FrontCenterRight",  
        "FrontAwayLeft",  "FrontAwayRight",  
        "BackAwayLeft",  "BackAwayRight",  
        "BackCenterLeft",  "BackCenterRight",  
        "BackProcessorLeft",  "BackProcessorRight"
    };

    for( size_t i=0; i<blueLeftReefPlacingPoses.size(); ++i ) {
        PP_poses.push_back( blueLeftReefPlacingPoses[i] );
        PP_poses.push_back( blueRightReefPlacingPoses[i] );
    }

    fmt::print( "{{\n  \"version\": \"2025.0\",\n  \"waypoints\": [\n");

    for( size_t i=0; i<PP_poses.size(); ++i ) {
        fmt::print("    {{\n");
        fmt::print("      \"anchor\": {{\n");
        fmt::print("        \"x\": {},\n", PP_poses[i].Translation().X().value() );
        fmt::print("        \"y\": {}\n", PP_poses[i].Translation().Y().value() );
        fmt::print("      }},\n");
        if( i == 0 ) {
            fmt::print("      \"prevControl\": null,\n");
        } else {
            fmt::print("      \"prevControl\": {{\n");
            fmt::print("        \"x\": {},\n", PP_poses[i].Translation().X().value() + 0.1 );
            fmt::print("        \"y\": {}\n", PP_poses[i].Translation().Y().value() );
            fmt::print("      }},\n");
        }
        if( i == PP_poses.size()-1 ) {
            fmt::print("      \"nextControl\": null,\n");
        } else {
            fmt::print("      \"nextControl\": {{\n");
            fmt::print("        \"x\": {},\n", PP_poses[i].Translation().X().value() - 0.1 );
            fmt::print("        \"y\": {}\n", PP_poses[i].Translation().Y().value() );
            fmt::print("      }},\n");
        }
        fmt::print("      \"isLocked\": true,\n");
        fmt::print("      \"linkedName\": \"{}\"\n", PP_names[i] );
        if( i == PP_poses.size()-1 ) {
            fmt::print("    }}\n" );
        } else {
            fmt::print("    }},\n" );
        }
    }

    fmt::print( "  ],\n");
    fmt::print( "  \"rotationTargets\": [],\n");
    fmt::print( "  \"constraintZones\": [],\n");
    fmt::print( "  \"pointTowardsZones\": [],\n");
    fmt::print( "  \"eventMarkers\": [],\n");
    fmt::print( "  \"globalConstraints\": {{\n");
    fmt::print( "    \"maxVelocity\": 3.0,\n");
    fmt::print( "    \"maxAcceleration\": 3.0,\n");
    fmt::print( "    \"maxAngularVelocity\": 540.0,\n");
    fmt::print( "    \"maxAngularAcceleration\": 720.0,\n");
    fmt::print( "    \"nominalVoltage\": 12.0,\n");
    fmt::print( "    \"unlimited\": false\n");
    fmt::print( "  }},\n");
    fmt::print( "  \"goalEndState\": {{\n");
    fmt::print( "    \"velocity\": 0.0,\n");
    fmt::print( "    \"rotation\": -119.99999999999999\n");
    fmt::print( "  }},\n");
    fmt::print( "  \"reversed\": false,\n");
    fmt::print( "  \"folder\": null,\n");
    fmt::print( "  \"idealStartingState\": {{\n");
    fmt::print( "    \"velocity\": 0.0,\n");
    fmt::print( "    \"rotation\": -59.99999999999999\n");
    fmt::print( "  }},\n");
    fmt::print( "  \"useDefaultConstraints\": true\n");
    fmt::print( "}}\n");
}

frc2::CommandPtr ReefCommands::PlaceOnReef( 
    Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide, std::function<ReefPlacement ()> place_func )
{
    return frc2::cmd::Sequence(
        frc2::cmd::Either( 
            frc2::cmd::Sequence(
                frc2::cmd::RunOnce( [intake] {intake->ShiftUpSlow();}, {intake}),
                DriveToReefPose( d, onRightSide ),
                intake->StopCmd(),
                frc2::cmd::Select<ReefPlacement>( 
                    [place_func] { return place_func(); }, 
                    // std::pair{ ReefPlacement::NONE, PlaceCoralNone() },
                    std::pair{ ReefPlacement::PLACING_L1, PlaceCoralL1( d, arm, intake, elevator ) },
                    std::pair{ ReefPlacement::PLACING_L2, PlaceCoralL2( d, arm, intake, elevator, onRightSide ) },
                    std::pair{ ReefPlacement::PLACING_L3, PlaceCoralL3( d, arm, intake, elevator, onRightSide ) },
                    std::pair{ ReefPlacement::PLACING_L4, PlaceCoralL4( d, arm, intake, elevator, onRightSide ) }
                ),
                frc2::cmd::Race(
                    IntakeCommands::RestPosition( arm, intake, elevator ),
                    frc2::cmd::Sequence(
                        frc2::cmd::WaitUntil( [elevator] { return elevator->GetHeight() < 36_in; }),
                        ControllerIO::JoystickDrive()
                    )
                )
            ),
            PlaceCoralNone(),
            [place_func] { return place_func() != ReefPlacement::NONE; }
        )
    ).WithName("PlaceOnReef");
}

frc2::CommandPtr ReefCommands::DriveToReefPose( Drive *d, bool onRightSide )
{
    return DriveToPoseTrap( d, [d, onRightSide] {
        return ReefCommands::reefPoses.GetClosestReefPose( d->GetPose(), onRightSide );
    },
    1.0
    ).WithName("DriveToReefPose");
}

frc2::CommandPtr ReefCommands::DriveToReefPoseDelta( Drive *d, frc::Transform2d move, bool onRightSide )
{
    return DriveToPoseTrap( d, [d, move, onRightSide] {
        frc::Pose2d reefPose = ReefCommands::reefPoses.GetClosestReefPose( d->GetPose(), onRightSide );
        return reefPose.TransformBy( move );
    },
    0.5
    ).WithName("DriveToReefPoseDelta");
}


frc2::CommandPtr ReefCommands::DriveToAlgaePose( Drive *d )
{
    return DriveToPoseTrap( d, [d] {
        return ReefCommands::reefPoses.GetClosestAlgaePose( d->GetPose() );
    },
    0.75
    ).WithName("DriveToAlgaePose");
}

frc2::CommandPtr ReefCommands::PlaceCoralNone( )
{
    return frc2::cmd::Parallel(
        frc2::cmd::Print( "No Reef Level Selected!!"),
        ControllerIO::NoLevelSelectedRumble()
    );
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
        DriveCommands::DriveDeltaPose( d, {reef::place_L1_shift_in, 0_in, 0_deg}, true, 0.5 ).WithTimeout( 1_s),
        intake->EjectCoralL1(),
        DriveCommands::DriveDeltaPose( d, {-8_in, 0_in, 0_deg}, true, 1.0 )
    ).WithName( "Place Coral in L1" );
}

frc2::CommandPtr ReefCommands::PlaceCoralL2( Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide )
{
    return frc2::cmd::Sequence(
        arm->ChangeElbowAngle( arm::kElbowForwardRaiseAngle ),
        frc2::cmd::Parallel(
            elevator->ChangeHeight( elevator::kHeightCoralL2 + 4_in ),
            frc2::cmd::WaitUntil( [elevator] { return elevator->GetHeight() > elevator::kHeightCoralL2 - 4_in;} )
                .AndThen( arm->ChangeElbowAndWrist( arm::kElbowCoralL2 - 14_deg, ArmIO::WristVertical ) )
        ),
        // elevator->ChangeHeight( elevator::kHeightCoralL2 ),
        // arm->ChangeWristPosition( ArmIO::WristVertical ),
        // arm->ChangeElbowAngle( arm::kElbowCoralL2 ),
        DriveToReefPoseDelta(  d, {reef::place_L2_shift_in, 0_in, 0_deg}, onRightSide ).WithTimeout( 1_s),
        frc2::cmd::Parallel(
            elevator->ChangeHeight( elevator::kHeightCoralL2 ),
            arm->ChangeElbowAngle( arm::kElbowCoralL2 )
        ),
        intake->EjectCoralL2_4( false ),
        frc2::cmd::Parallel(
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            arm->ChangeElbowAngle( arm::kElbowCoralL2 - 5_deg ).WithTimeout(0.1_s),
            elevator->ChangeHeight( elevator::kHeightCoralL2 - 3_in )
        ),
        DriveCommands::DriveDeltaPose( d, {-15_in, 0_in, 0_deg}, true, 1.0 ),
        frc2::cmd::RunOnce( [intake] { intake->Stop(); })
    ).WithName( "Place Coral in L2" );
}

frc2::CommandPtr ReefCommands::PlaceCoralL3( Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide )
{
    return frc2::cmd::Sequence(
        arm->ChangeElbowAngle( arm::kElbowForwardRaiseAngle ),
        frc2::cmd::Parallel(
            elevator->ChangeHeight( elevator::kHeightCoralL3 + 4_in ),
            frc2::cmd::WaitUntil( [elevator] { return elevator->GetHeight() > elevator::kHeightCoralL3 - 12_in;} )
                .AndThen( arm->ChangeElbowAndWrist( arm::kElbowCoralL3 - 14_deg, ArmIO::WristVertical ) )
        ),
        // elevator->ChangeHeight( elevator::kHeightCoralL3 ),
        // arm->ChangeWristPosition( ArmIO::WristVertical ),
        // arm->ChangeElbowAngle( arm::kElbowCoralL3 ),
        DriveToReefPoseDelta( d, {reef::place_L3_shift_in, 0_in, 0_deg}, onRightSide ).WithTimeout(1.5_s),
        frc2::cmd::Parallel(
            elevator->ChangeHeight( elevator::kHeightCoralL3 ),
            arm->ChangeElbowAngle( arm::kElbowCoralL3 )
        ),
        intake->EjectCoralL2_4( false ),
        frc2::cmd::Parallel(
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            arm->ChangeElbowAngle( arm::kElbowCoralL3 - 5_deg ).WithTimeout(0.1_s),
            elevator->ChangeHeight( elevator::kHeightCoralL3 - 3_in )
        ),
        DriveCommands::DriveDeltaPose( d, {-15_in, 0_in, 0_deg}, true, 1.0 ),
        intake->StopCmd()
        // frc2::cmd::Parallel(
        //     intake->EjectCoralL2_4( true ),
        //     DriveCommands::DriveOpenLoop( d, {1_fps, 0_fps, 0_rpm}, true ).WithTimeout( 1_s)
        // )
    ).WithName( "Place Coral in L3" );
}

frc2::CommandPtr ReefCommands::PlaceCoralL4( Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide )
{
    return frc2::cmd::Sequence(
        arm->ChangeElbowAngle( arm::kElbowForwardRaiseAngle ),
        frc2::cmd::Parallel(
            elevator->ChangeHeight( elevator::kHeightCoralL4 ),
            frc2::cmd::WaitUntil( [elevator] { return elevator->GetHeight() > elevator::kHeightCoralL4 - 20_in;} )
                .AndThen( arm->ChangeElbowAndWrist( arm::kElbowCoralL4, ArmIO::WristVertical ) )
        ),
        DriveToReefPoseDelta( d, {reef::place_L4_shift_in, 0_in, 0_deg}, onRightSide ).WithTimeout( 1_s),
        intake->EjectCoralL2_4( false ),
        frc2::cmd::Parallel(
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            arm->ChangeElbowAngle( arm::kElbowCoralL4 - 10_deg ).WithTimeout(0.1_s),
            elevator->ChangeHeight( elevator::kHeightCoralL4 - 2_in )
        ),
        DriveCommands::DriveDeltaPose( d, {-15_in, 0_in, 0_deg}, true, 1.0 ),
        frc2::cmd::RunOnce( [intake] { intake->Stop(); })
        // frc2::cmd::Parallel(
        //     intake->EjectCoralL2_4( true ),
        //     DriveCommands::DriveOpenLoop( d, {1.1_fps, 0_fps, 0_rpm}, true ).WithTimeout( 1_s)
        // )
    ).WithName( "Place Coral in L4" );
}

frc2::CommandPtr ReefCommands::RemoveAlgae( Drive *d, Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence( 
        DriveToAlgaePose( d ),
        arm->ChangeElbowAngle( arm::kElbowForwardRaiseAngle ),
        frc2::cmd::RunOnce( [intake] {intake->ShiftDown();}, {intake} ),
        frc2::cmd::Either( 
            frc2::cmd::Sequence(
                // Low Algae
                elevator->ChangeHeight( elevator::kHeightRemoveAlgaeLow ),
                arm->ChangeElbowAngle( arm::kElbowRemoveAlgaeLow )
            ),
            // High Algae
            frc2::cmd::Sequence(
                elevator->ChangeHeight( elevator::kHeightRemoveAlgaeHigh ),
                arm->ChangeElbowAngle( arm::kElbowRemoveAlgaeHigh )
            ),
            [d] { return ReefCommands::reefPoses.isAlgaeLow( d->GetPose() ); }
        ),
        DriveCommands::DriveDeltaPose( d, {reef::algae_shift_in, 0_in, 0_deg}, true, 0.5 ),
        frc2::cmd::Either( 
            elevator->ChangeHeight( elevator::kHeightRemoveAlgaeLow + 4_in ),
            elevator->ChangeHeight( elevator::kHeightRemoveAlgaeHigh + 4_in ),
            [d] { return ReefCommands::reefPoses.isAlgaeLow( d->GetPose() ); }
        ),
        frc2::cmd::Wait( 0.8_s ),
        frc2::cmd::RunOnce( [intake] {intake->Stop();}, {intake} ),
        frc2::cmd::Parallel(
            DriveCommands::DriveDeltaPose( d, {-reef::algae_shift_in, 0_in, 0_deg}, true, 0.5 ),
            arm->ChangeElbowAngle( arm::kElbowAlgaeHoldingPos ),
            elevator->ChangeHeight( elevator::kHeightRemoveAlgaeLow + 4_in )
        )
    );
}

frc2::CommandPtr ReefCommands::PrepareToClimb( Arm *arm, Climber *climber )
{
    return frc2::cmd::Parallel( 
        DriveCommands::SetDriveSpeed( true ),
        arm->ChangeElbowAngle( arm::kElbowGroundPickup ),
        climber->RaiseClimber()
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

