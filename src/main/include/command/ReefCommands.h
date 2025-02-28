#pragma once

#include <frc2/command/CommandPtr.h>

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

class ReefPlacingPoses {
public:
    ReefPlacingPoses();

    frc::Pose2d GetClosest( frc::Pose2d currentPose, bool onRightSide );

private:
    frc::AprilTagFieldLayout aprilTags;
    std::vector<frc::Pose2d> redLeftReefPlacingPoses;
    std::vector<frc::Pose2d> redRightReefPlacingPoses;
    std::vector<frc::Pose2d> blueLeftReefPlacingPoses;
    std::vector<frc::Pose2d> blueRightReefPlacingPoses;
};

class ReefCommands {
public:

    static frc2::CommandPtr PlaceOnReef( Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide );
    static frc2::CommandPtr DriveToReefPose( Drive *d, bool onRightSide );
    static frc2::CommandPtr PlaceCoralL1( Drive *, Arm *, Intake *, Elevator * );
    static frc2::CommandPtr PlaceCoralL2( Drive *, Arm *, Intake *, Elevator * );
    static frc2::CommandPtr PlaceCoralL3( Drive *, Arm *, Intake *, Elevator * );
    static frc2::CommandPtr PlaceCoralL4( Drive *, Arm *, Intake *, Elevator * );

    static frc2::CommandPtr RemoveAlgae( Arm *, Intake *, Elevator * );

    static frc2::CommandPtr SetReefPlacement( ReefPlacement );

private:
    ReefCommands() = default;
    static void LogReefPlacement( ReefPlacement );

    static ReefPlacingPoses reefPoses;
    static ReefPlacement next_reef_place;
};