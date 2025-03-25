#pragma once

#include <vector> 

#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandPtr.h>

#include "command/ReefCommands.h"

class Arm;
class Drive;
class Intake;
class Elevator;

class AutoCommands {
public:

    static frc2::CommandPtr PrepareToPlaceOnReef( Arm *arm, Elevator *elevator, ReefPlacement p );
    static frc2::CommandPtr ReefToCoralStation( Arm *arm, Intake *intake, Elevator *elevator );
    static frc2::CommandPtr CoralStationPickup( Arm *arm, Intake *intake, Elevator *elevator );
    static frc2::CommandPtr LeaveCoralStation( Arm *arm, Intake *intake, Elevator *elevator, ReefPlacement p );
    static frc2::CommandPtr AutoPlaceCoralL1( Intake *intake );
    static frc2::CommandPtr AutoPlaceCoralL2( Arm *arm, Intake *intake, Elevator *elevator );
    static frc2::CommandPtr AutoPlaceCoralL3( Arm *arm, Intake *intake, Elevator *elevator );
    static frc2::CommandPtr AutoPlaceCoralL4( Arm *arm, Intake *intake, Elevator *elevator );
    static frc2::CommandPtr AutoEndAtReef( Drive *d, Arm *arm, Intake *intake, Elevator *elevator );


private:
    AutoCommands() = default;
};