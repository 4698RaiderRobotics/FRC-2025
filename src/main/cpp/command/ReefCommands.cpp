
#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "command/ReefCommands.h"
#include "command/IntakeCommands.h"
#include "command/DriveCommands.h"
#include "command/AutoCommands.h"
#include "command/DriveToPose.h"
#include "command/MoveMechanism.h"
#include "command/ControllerIO.h"

#include "arm/Arm.h"
#include "swerve/Drive.h"
#include "intake/Intake.h"
#include "elevator/Elevator.h"
#include "climber/Climber.h"

#include "util/DataLogger.h"

#include "Constants.h"

using namespace physical;

void WritePathPlannerJSONFile( std::string fname, std::vector<frc::Pose2d> &PP_poses, std::vector<std::string> &PP_names );

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

    std::vector<std::string> PP_names = {
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

    WritePathPlannerJSONFile( "ReefPoints.path", PP_poses, PP_names );

    PP_poses.clear();
    PP_names = {
        "FrontProcessorLeftL4",  "FrontProcessorRightL4",  
        "FrontCenterLeftL4",  "FrontCenterRightL4",  
        "FrontAwayLeftL4",  "FrontAwayRightL4",  
        "BackAwayLeftL4",  "BackAwayRightL4",  
        "BackCenterLeftL4",  "BackCenterRightL4",  
        "BackProcessorLeftL4",  "BackProcessorRightL4"
    };

    for( size_t i=0; i<blueLeftReefPlacingPoses.size(); ++i ) {
        PP_poses.push_back( blueLeftReefPlacingPoses[i].TransformBy( {reef::place_L4_shift_in, 0_in, 0_deg} ) );
        PP_poses.push_back( blueRightReefPlacingPoses[i].TransformBy( {reef::place_L4_shift_in, 0_in, 0_deg} ) );
    }

    WritePathPlannerJSONFile( "ReefPointsL4.path", PP_poses, PP_names );
}

void WritePathPlannerJSONFile( std::string fname, std::vector<frc::Pose2d> &PP_poses, std::vector<std::string> &PP_names ) 
{
    FILE *fp = fopen( fname.c_str(), "wt" );

    fmt::print( fp, "{{\n  \"version\": \"2025.0\",\n  \"waypoints\": [\n");

    for( size_t i=0; i<PP_poses.size(); ++i ) {
        fmt::print(fp, "    {{\n");
        fmt::print(fp, "      \"anchor\": {{\n");
        fmt::print(fp, "        \"x\": {},\n", PP_poses[i].Translation().X().value() );
        fmt::print(fp, "        \"y\": {}\n", PP_poses[i].Translation().Y().value() );
        fmt::print(fp, "      }},\n");
        if( i == 0 ) {
            fmt::print(fp, "      \"prevControl\": null,\n");
        } else {
            fmt::print(fp, "      \"prevControl\": {{\n");
            fmt::print(fp, "        \"x\": {},\n", PP_poses[i].Translation().X().value() + 0.1 );
            fmt::print(fp, "        \"y\": {}\n", PP_poses[i].Translation().Y().value() );
            fmt::print(fp, "      }},\n");
        }
        if( i == PP_poses.size()-1 ) {
            fmt::print(fp, "      \"nextControl\": null,\n");
        } else {
            fmt::print(fp, "      \"nextControl\": {{\n");
            fmt::print(fp, "        \"x\": {},\n", PP_poses[i].Translation().X().value() - 0.1 );
            fmt::print(fp, "        \"y\": {}\n", PP_poses[i].Translation().Y().value() );
            fmt::print(fp, "      }},\n");
        }
        fmt::print(fp, "      \"isLocked\": true,\n");
        fmt::print(fp, "      \"linkedName\": \"{}\"\n", PP_names[i] );
        if( i == PP_poses.size()-1 ) {
            fmt::print(fp, "    }}\n" );
        } else {
            fmt::print(fp, "    }},\n" );
        }
    }

    fmt::print( fp, "  ],\n");
    fmt::print( fp, "  \"rotationTargets\": [],\n");
    fmt::print( fp, "  \"constraintZones\": [],\n");
    fmt::print( fp, "  \"pointTowardsZones\": [],\n");
    fmt::print( fp, "  \"eventMarkers\": [],\n");
    fmt::print( fp, "  \"globalConstraints\": {{\n");
    fmt::print( fp, "    \"maxVelocity\": 3.0,\n");
    fmt::print( fp, "    \"maxAcceleration\": 3.0,\n");
    fmt::print( fp, "    \"maxAngularVelocity\": 540.0,\n");
    fmt::print( fp, "    \"maxAngularAcceleration\": 720.0,\n");
    fmt::print( fp, "    \"nominalVoltage\": 12.0,\n");
    fmt::print( fp, "    \"unlimited\": false\n");
    fmt::print( fp, "  }},\n");
    fmt::print( fp, "  \"goalEndState\": {{\n");
    fmt::print( fp, "    \"velocity\": 0.0,\n");
    fmt::print( fp, "    \"rotation\": -119.99999999999999\n");
    fmt::print( fp, "  }},\n");
    fmt::print( fp, "  \"reversed\": false,\n");
    fmt::print( fp, "  \"folder\": null,\n");
    fmt::print( fp, "  \"idealStartingState\": {{\n");
    fmt::print( fp, "    \"velocity\": 0.0,\n");
    fmt::print( fp, "    \"rotation\": -59.99999999999999\n");
    fmt::print( fp, "  }},\n");
    fmt::print( fp, "  \"useDefaultConstraints\": true\n");
    fmt::print( fp, "}}\n");

    fclose( fp );
}

frc2::CommandPtr ReefCommands::CancelAll( Arm *arm, Intake *intake, Elevator *elevator, Climber *climber )
{
    return frc2::cmd::Sequence(
        frc2::cmd::Either( 
            frc2::cmd::None(),
            IntakeCommands::RestPosition( arm, intake, elevator ),
            [climber] { return climber->DoingSequence(); }
        ),
        climber->StopClimber()
    ).WithName("Cancel All");
}

frc2::CommandPtr ReefCommands::PlaceOnReef( 
    Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide, std::function<ReefPlacement ()> place_func )
{
    return frc2::cmd::Sequence(
        frc2::cmd::RunOnce( [intake] {intake->Stop();}, {intake} ),
        frc2::cmd::Either( 
            frc2::cmd::Sequence(
                frc2::cmd::RunOnce( [intake] {intake->enableRetention(false);}),
                frc2::cmd::Select<ReefPlacement>( 
                    [place_func] { return place_func(); }, 
                    std::pair{ ReefPlacement::PLACING_L1, frc2::cmd::Parallel(
                        PrePlaceCoralL1( arm, elevator ),
                        DriveToReefPose( d, onRightSide, place_func )
                    )},
                    std::pair{ ReefPlacement::PLACING_L2,frc2::cmd::Parallel(
                        PrePlaceCoralL2( arm, elevator ),
                        DriveToReefPose( d, onRightSide, place_func )
                    )},
                    std::pair{ ReefPlacement::PLACING_L3, frc2::cmd::Parallel(
                        PrePlaceCoralL3( arm, elevator ),
                        DriveToReefPose( d, onRightSide, place_func )
                    )},
                    std::pair{ ReefPlacement::PLACING_L4, frc2::cmd::Parallel(
                        PrePlaceCoralL4( arm, elevator ),
                        DriveToReefPose( d, onRightSide, place_func )
                    )}
                ),
                intake->StopCmd(),
                frc2::cmd::Select<ReefPlacement>( 
                    [place_func] { return place_func(); }, 
                    std::pair{ ReefPlacement::PLACING_L1, PlaceCoralL1( d, arm, intake, elevator, onRightSide ) },
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
                ),
                frc2::cmd::RunOnce( [intake] {intake->enableRetention(true);})
            ),
            PlaceCoralNone(),
            [place_func] { return place_func() != ReefPlacement::NONE; }
        )
    ).WithName("PlaceOnReef");
}

frc2::CommandPtr ReefCommands::PrepareToPlaceOnReef( Arm *arm, Elevator *elevator, std::function<ReefPlacement ()> place_func )
{
    return frc2::cmd::Select<ReefPlacement>( 
        [place_func] { return place_func(); }, 
        std::pair{ ReefPlacement::PLACING_L1, PrePlaceCoralL1( arm, elevator ) },
        std::pair{ ReefPlacement::PLACING_L2, PrePlaceCoralL2( arm, elevator ) },
        std::pair{ ReefPlacement::PLACING_L3, PrePlaceCoralL3( arm, elevator ) },
        std::pair{ ReefPlacement::PLACING_L4, PrePlaceCoralL4( arm, elevator ) } 
    );
}

frc2::CommandPtr ReefCommands::DriveToReefPose( Drive *d, bool onRightSide, std::function<ReefPlacement ()> place_func )
{
    return DriveCommands::DriveToPosePP( d, [d, onRightSide, place_func] {
        frc::Transform2d delta, mid_delta;

        switch( place_func() ) {
        case ReefPlacement::PLACING_L1:
            delta = {reef::place_L1_shift_in, 0_in, 0_deg};
            break;
        case ReefPlacement::PLACING_L2:
            delta = {reef::place_L2_shift_in, 0_in, 0_deg};
            break;
        case ReefPlacement::PLACING_L3:
            delta = {reef::place_L3_shift_in, 0_in, 0_deg};
            break;
        case ReefPlacement::PLACING_L4:
            delta = {reef::place_L4_shift_in, 0_in, 0_deg};
            break;
        default:
            delta = {0_in, 0_in, 0_deg};
            break;
        }

        mid_delta = {delta.X() - 18_in, 0_in, 0_deg};

        // Use the chassis speed to project the robots pose into the slight future
        frc::Pose2d currentPose = d->GetPose();
        frc::Pose2d lookAheadPose = currentPose.Exp( d->GetChassisSpeeds().ToTwist2d( 200_ms ) );
        frc::Pose2d reefPose = ReefCommands::reefPoses.GetClosestReefPose( lookAheadPose, onRightSide );

        DataLogger::Log( "DriveToReefPose/currentPose", currentPose );
        DataLogger::Log( "DriveToReefPose/lookAheadPose", lookAheadPose );
        DataLogger::Log( "DriveToReefPose/reefPose", reefPose );
        
        return std::vector<frc::Pose2d> {reefPose.TransformBy( delta ), reefPose.TransformBy( mid_delta )};
    },
    place_func
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

frc2::CommandPtr ReefCommands::PrePlaceCoralL1( Arm *arm, Elevator *elevator )
{
    return MoveMechanism( arm, elevator, elevator::kHeightCoralL1, arm::kElbowCoralL1, ArmIO::WristHorizontal ).ToPtr();
}

frc2::CommandPtr ReefCommands::PlaceCoralL1( Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide )
{
    return frc2::cmd::Sequence(
        intake->EjectCoralL1(),
        DriveCommands::DriveDeltaPose( d, {-reef::place_L1_shift_in, 0_in, 0_deg}, true, 1.0 ),
        intake->StopCmd()
    ).WithName( "Place Coral in L1" );
}

frc2::CommandPtr ReefCommands::PrePlaceCoralL2( Arm *arm, Elevator *elevator )
{
    return MoveMechanism( arm, elevator, elevator::kHeightCoralL2, arm::kElbowCoralL2, ArmIO::WristVertical ).ToPtr();
}

frc2::CommandPtr ReefCommands::PlaceCoralL2( Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide )
{
    return frc2::cmd::Sequence(
        intake->EjectCoralL2_4_Fast(),
        frc2::cmd::Parallel(
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            arm->ChangeElbowAngle( arm::kElbowCoralL2 - 5_deg ).WithTimeout(0.1_s),
            elevator->ChangeHeight( elevator::kHeightCoralL2 - 3_in )
        ),
        DriveCommands::DriveDeltaPose( d, {-reef::place_L2_shift_in, 0_in, 0_deg}, true, 1.0 ),
        frc2::cmd::RunOnce( [intake] { intake->Stop(); })
    ).WithName( "Place Coral in L2" );
}

frc2::CommandPtr ReefCommands::PrePlaceCoralL3( Arm *arm, Elevator *elevator )
{
    return MoveMechanism( arm, elevator, elevator::kHeightCoralL3, arm::kElbowCoralL3, ArmIO::WristVertical ).ToPtr();
}

frc2::CommandPtr ReefCommands::PlaceCoralL3( Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide )
{
    return frc2::cmd::Sequence(
        intake->EjectCoralL2_4_Fast(),
        frc2::cmd::Parallel(
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            arm->ChangeElbowAngle( arm::kElbowCoralL3 - 5_deg ).WithTimeout(0.1_s),
            elevator->ChangeHeight( elevator::kHeightCoralL3 - 3_in )
        ),
        DriveCommands::DriveDeltaPose( d, {-reef::place_L3_shift_in, 0_in, 0_deg}, true, 1.0 ),
        intake->StopCmd()
    ).WithName( "Place Coral in L3" );
}

frc2::CommandPtr ReefCommands::PrePlaceCoralL4( Arm *arm, Elevator *elevator )
{    
    return MoveMechanism( arm, elevator, elevator::kHeightCoralL4, arm::kElbowCoralL4, ArmIO::WristVertical ).ToPtr();
}

frc2::CommandPtr ReefCommands::PlaceCoralL4( Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide )
{
    return frc2::cmd::Sequence(
        intake->EjectCoralL2_4_Fast(),
        frc2::cmd::Parallel(
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            arm->ChangeElbowAngle( arm::kElbowCoralL4 - 10_deg ).WithTimeout(0.1_s),
            elevator->ChangeHeight( elevator::kHeightCoralL4 - 2_in )
        ),
        DriveCommands::DriveDeltaPose( d, {-reef::place_L4_shift_in -2_in, 0_in, 0_deg}, true, 0.75 ),
        frc2::cmd::RunOnce( [intake] { intake->Stop(); })
    ).WithName( "Place Coral in L4" );
}

frc2::CommandPtr ReefCommands::RemoveAlgae( Drive *d, Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence( 
        frc2::cmd::Either( 
            frc2::cmd::Sequence(
                // Low Algae
                frc2::cmd::Parallel(
                    DriveToAlgaePose( d ),
                    MoveMechanism( arm, elevator, elevator::kHeightRemoveAlgaeLow, arm::kElbowRemoveAlgaeLow, ArmIO::WristHorizontal ).ToPtr()
                ),
                frc2::cmd::RunOnce( [intake] {intake->ShiftDown();}, {intake} ),
                DriveCommands::DriveDeltaPose( d, {reef::algae_shift_in - 2.5_in, 0_in, 0_deg}, true, 0.5 )
            ),
            // High Algae
            frc2::cmd::Sequence(
                frc2::cmd::Parallel(
                    DriveToAlgaePose( d ),
                    MoveMechanism( arm, elevator, elevator::kHeightRemoveAlgaeHigh, arm::kElbowRemoveAlgaeHigh, ArmIO::WristHorizontal ).ToPtr()
                ),
                frc2::cmd::RunOnce( [intake] {intake->ShiftDown();}, {intake} ),
                DriveCommands::DriveDeltaPose( d, {reef::algae_shift_in - 1_in, 0_in, 0_deg}, true, 0.5 )
            ),
            [d] { return ReefCommands::reefPoses.isAlgaeLow( d->GetPose() ); }
        ),
        frc2::cmd::Either(
            MoveMechanism( arm, elevator, elevator::kHeightRemoveAlgaeLow + 2_in, arm::kElbowRemoveAlgaeHigh + 7_deg, ArmIO::WristHorizontal ).ToPtr(),
            MoveMechanism( arm, elevator, elevator::kHeightRemoveAlgaeHigh + 4_in, arm::kElbowRemoveAlgaeHigh + 5_deg, ArmIO::WristHorizontal ).ToPtr(),
            [d] { return ReefCommands::reefPoses.isAlgaeLow( d->GetPose() ); }
        ),
        frc2::cmd::Wait( 0.6_s ),
        frc2::cmd::Parallel(
            frc2::cmd::RunOnce( [intake] {intake->Stop();}, {intake} ),
            DriveCommands::DriveDeltaPose( d, {-reef::algae_shift_in, 0_in, 0_deg}, true, 0.5 )
        )
    );

}

frc2::CommandPtr ReefCommands::LockClimberToCage( Arm *arm, Climber *climber )
{
    return frc2::cmd::Sequence( 
        frc2::cmd::Parallel( 
            // DriveCommands::SetDriveSpeed( true ),
            arm->ChangeElbowAngle( arm::kElbowGroundPickup ),
            climber->RaiseClimber()
        ),
        frc2::cmd::WaitUntil( [climber] { return climber->CageLockedIn(); } ),
        climber->DoClimb()
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

