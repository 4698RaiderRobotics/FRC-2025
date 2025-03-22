#pragma once

#include <vector> 

#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc/apriltag/AprilTagFieldLayout.h>

enum class ReefPlacement {
    NONE,
    PLACING_L1,
    PLACING_L2,
    PLACING_L3,
    PLACING_L4,
};

class Arm;
class Drive;
class Intake;
class Elevator;
class Climber;

class ReefPlacingPoses {
public:
    ReefPlacingPoses();

    frc::Pose2d GetClosestReefPose( frc::Pose2d currentPose, bool onRightSide );
    frc::Pose2d GetClosestAlgaePose( frc::Pose2d currentPose );
    bool isAlgaeLow( frc::Pose2d currentPose );

    void OutputPathPlannerJSON();

private:
    frc::AprilTagFieldLayout aprilTags;

    std::vector<frc::Pose2d> redAlgaeRemovingPoses;
    std::vector<frc::Pose2d> redLeftReefPlacingPoses;
    std::vector<frc::Pose2d> redRightReefPlacingPoses;

    std::vector<frc::Pose2d> blueAlgaeRemovingPoses;
    std::vector<frc::Pose2d> blueLeftReefPlacingPoses;
    std::vector<frc::Pose2d> blueRightReefPlacingPoses;
};

class ReefCommands {
public:

    static frc2::CommandPtr PlaceOnReef( 
        Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide, std::function<ReefPlacement ()> place_func );
    static frc2::CommandPtr PrepareToPlaceOnReef( Arm *arm, Elevator *elevator, std::function<ReefPlacement ()> place_func );

    static frc2::CommandPtr DriveToReefPose( Drive *d, bool onRightSide, std::function<ReefPlacement ()> place_func );
    static frc2::CommandPtr DriveToReefPoseDelta( Drive *d, frc::Transform2d move, bool onRightSide );
    static frc2::CommandPtr DriveToAlgaePose( Drive *d );
    static frc2::CommandPtr PlaceCoralNone( );

    static frc2::CommandPtr PrePlaceCoralL1( Arm *arm, Elevator *elevator );
    static frc2::CommandPtr PrePlaceCoralL2( Arm *arm, Elevator *elevator );
    static frc2::CommandPtr PrePlaceCoralL3( Arm *arm, Elevator *elevator );
    static frc2::CommandPtr PrePlaceCoralL4( Arm *arm, Elevator *elevator );

    static frc2::CommandPtr PlaceCoralL1( Drive *, Arm *, Intake *, Elevator *, bool onRightSide );
    static frc2::CommandPtr PlaceCoralL2( Drive *, Arm *, Intake *, Elevator *, bool onRightSide );
    static frc2::CommandPtr PlaceCoralL3( Drive *, Arm *, Intake *, Elevator *, bool onRightSide );
    static frc2::CommandPtr PlaceCoralL4( Drive *, Arm *, Intake *, Elevator *, bool onRightSide );

    static frc2::CommandPtr RemoveAlgae( Drive *d, Arm *arm, Intake *intake, Elevator *elevator );

    static frc2::CommandPtr PrepareToClimb( Arm *arm, Climber *climber );

private:
    ReefCommands() = default;

    static ReefPlacingPoses reefPoses;
};