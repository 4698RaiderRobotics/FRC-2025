#pragma once

#include <frc2/command/CommandPtr.h>

class Arm;
class Drive;
class Intake;
class Elevator;

class IntakeCommands {
public:

    static frc2::CommandPtr CoralStationPickup( Arm *arm, Intake *intake, Elevator *elevator );
    static frc2::CommandPtr GroundPickup( Arm *arm, Intake *intake, Elevator *elevator );

private:
    IntakeCommands() = default;
};