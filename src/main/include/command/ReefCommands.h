#pragma once

#include <frc2/command/CommandPtr.h>

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

    static frc2::CommandPtr DriveToReefPose( Drive *d, bool onRightSide );
    static frc2::CommandPtr PlaceCoralL1( Arm *, Intake *, Elevator * );
    static frc2::CommandPtr PlaceCoralL2( Arm *, Intake *, Elevator * );
    static frc2::CommandPtr PlaceCoralL3( Arm *, Intake *, Elevator * );
    static frc2::CommandPtr PlaceCoralL4( Arm *, Intake *, Elevator * );

    static frc2::CommandPtr RemoveAlgae( Arm *, Intake *, Elevator * );

private:
    ReefCommands() = default;

    static ReefPlacingPoses reefPoses;

};