#pragma once

#include <frc2/command/CommandPtr.h>

class Arm;
class Intake;
class Elevator;

class ReefCommands {
public:
    static frc2::CommandPtr PlaceCoralL1( Arm *, Intake *, Elevator * );
    static frc2::CommandPtr PlaceCoralL2( Arm *, Intake *, Elevator * );
    static frc2::CommandPtr PlaceCoralL3( Arm *, Intake *, Elevator * );
    static frc2::CommandPtr PlaceCoralL4( Arm *, Intake *, Elevator * );

    static frc2::CommandPtr RemoveAlgae( Arm *, Intake *, Elevator * );

private:
    ReefCommands() = default;
};