
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "command/IntakeCommands.h"

#include "arm/Arm.h"
#include "swerve/Drive.h"
#include "intake/Intake.h"
#include "elevator/Elevator.h"

#include "Constants.h"

using namespace physical;

frc2::CommandPtr IntakeCommands::CoralStationPickup( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        arm->ChangeElbowAngle( arm::kElbowRaiseAngle ),
        arm->ChangeWristPosition( ArmIO::WristHorizontal ),  
        frc2::cmd::Parallel(
            elevator->ChangeHeight( elevator::kHeightCoralStation ),
            arm->ChangeElbowAngle( arm::kElbowCoralStation )
        ),
        intake->IntakeCoral()
    ).WithName( "Coral Station Pickup" );
}

frc2::CommandPtr IntakeCommands::GroundPickup( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        arm->ChangeElbowAngle( arm::kElbowRaiseAngle ),
        arm->ChangeWristPosition( ArmIO::WristHorizontal ),  
        frc2::cmd::Parallel(
            elevator->ChangeHeight( elevator::kHeightGroundPickup ),
            arm->ChangeElbowAngle( arm::kElbowGroundPickup )
        ),
        intake->IntakeCoral()
    ).WithName( "Ground Pickup" );
}

