
#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "command/ReefCommands.h"

#include "arm/Arm.h"
#include "intake/Intake.h"
#include "elevator/Elevator.h"

#include "Constants.h"

using namespace physical;

frc2::CommandPtr ElevatorRaisePosition( Arm *arm )
{
    return frc2::cmd::Parallel(
        arm->ChangeElbowAngle( arm::kElbowRaiseAngle ),
        arm->ChangeWristPosition( ArmIO::WristHorizontal )
    );
}

frc2::CommandPtr ReefCommands::PlaceCoralL1( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        ElevatorRaisePosition( arm ),
        elevator->ChangeHeight( elevator::kCoralL1Height ),
        arm->ChangeElbowAngle( arm::kElbowCoralL1 ),
        intake->EjectCoral()
    ).WithName( "Place Coral in L1" );
}