
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "command/IntakeCommands.h"
#include "command/MoveMechanism.h"

#include "arm/Arm.h"
#include "swerve/Drive.h"
#include "intake/Intake.h"
#include "elevator/Elevator.h"

#include "Constants.h"

using namespace physical;

frc2::CommandPtr IntakeCommands::RestPosition( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        MoveMechanism( arm, elevator, elevator::kElevatorMinHeight, arm::kElbowRestAngle, ArmIO::WristHorizontal ).ToPtr(),
        frc2::cmd::RunOnce( [intake] {intake->Stop();}, {intake} )
    );
}

frc2::CommandPtr IntakeCommands::CoralStationPickup( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        MoveMechanism( arm, elevator, elevator::kHeightCoralStation, arm::kElbowCoralStation, ArmIO::WristHorizontal ).ToPtr(),
        intake->IntakeCoral(),
        RestPosition( arm, intake, elevator )
    ).WithName( "Coral Station Pickup" );
}

frc2::CommandPtr IntakeCommands::CoralStationResume( Arm *arm, Intake *intake, Elevator *elevator, bool eject )
{
    return frc2::cmd::Sequence(
        MoveMechanism( arm, elevator, elevator::kHeightCoralStation, arm::kElbowCoralStation, ArmIO::WristHorizontal ).ToPtr(),
        // elevator->ChangeHeight( elevator::kHeightCoralStation ),
        // arm->ChangeElbowAngle( arm::kElbowCoralStation ),
        frc2::cmd::Either( 
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            intake->IntakeCoral(),
            [eject] { return eject; }
        )
    ).WithName( "Coral Station Resume" );
}

frc2::CommandPtr IntakeCommands::GroundPickup( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        MoveMechanism( arm, elevator, elevator::kHeightGroundPickup, arm::kElbowGroundPickup, ArmIO::WristHorizontal ).ToPtr(),
        // arm->ChangeWristPosition( ArmIO::WristHorizontal ),  
        // frc2::cmd::Parallel(
        //     elevator->ChangeHeight( elevator::kHeightGroundPickup ),
        //     arm->ChangeElbowAngle( arm::kElbowGroundPickup )
        // ),
        intake->IntakeCoral(),
        frc2::cmd::Race(
            MoveMechanism( arm, elevator, elevator::kElevatorMinHeight, arm::kElbowRestAngle, ArmIO::WristHorizontal ).ToPtr(),
            intake->IndexCoral()
        ),
        intake->StopCmd()
    ).WithName( "Ground Pickup" );
}

frc2::CommandPtr IntakeCommands::GroundResume( Arm *arm, Intake *intake, Elevator *elevator, bool eject )
{
    return frc2::cmd::Sequence(
        MoveMechanism( arm, elevator, elevator::kHeightGroundPickup, arm::kElbowGroundPickup, ArmIO::WristHorizontal ).ToPtr(),
        // elevator->ChangeHeight( elevator::kHeightGroundPickup ),
        // arm->ChangeElbowAngle( arm::kElbowGroundPickup ),
        frc2::cmd::Either( 
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            intake->IntakeCoral(),
            [eject] { return eject; }
        )
    ).WithName( "Ground Resume" );
}

frc2::CommandPtr IntakeCommands::ElevatorRaise( Arm *arm, Elevator *elevator )
{
    return // frc2::cmd::Sequence( 
        MoveMechanism( arm, elevator, elevator::kHeightDislogAlgae, arm::kElbowForwardRaiseAngle, ArmIO::WristHorizontal ).ToPtr();
        // arm->ChangeElbowAngle( arm::kElbowForwardRaiseAngle ),
        // elevator->ChangeHeight( elevator::kHeightDislogAlgae )
    // );
}       
