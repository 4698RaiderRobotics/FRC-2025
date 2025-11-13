#pragma once

#include <frc2/command/CommandPtr.h>

class Arm;
class Drive;
class Intake;
class Elevator;

class IntakeCommands {
public:

    static frc2::CommandPtr RestPosition( Arm *arm, Intake *intake, Elevator *elevator );
    static frc2::CommandPtr CoralHoldPos( Arm *arm, Intake *intake, Elevator *elevator );

    static frc2::CommandPtr CoralStationPickup( Arm *arm, Intake *intake, Elevator *elevator );
    static frc2::CommandPtr CoralStationResume( Arm *arm, Intake *intake, Elevator *elevator, bool eject );
    static frc2::CommandPtr GroundPickup( Arm *arm, Intake *intake, Elevator *elevator );
    static frc2::CommandPtr GroundResume( Arm *arm, Intake *intake, Elevator *elevator, bool eject );
    static frc2::CommandPtr ElevatorRaise( Arm *arm, Elevator *elevator );
    static frc2::CommandPtr ManualPlace3( Arm *arm, Intake *intake, Elevator *elevator );
    static frc2::CommandPtr ManualPlace2( Arm *arm, Intake *intake, Elevator *elevator );

private:
    IntakeCommands() = default;
};