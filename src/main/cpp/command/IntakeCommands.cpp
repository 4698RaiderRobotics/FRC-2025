
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

frc2::CommandPtr IntakeCommands::CoralHoldPos( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Parallel(
        frc2::cmd::Either(
                // Has Coral, index it
            intake->IndexCoral(),
                // No Coral, eject any Coral partly in intake
            frc2::cmd::Sequence(
                intake->EjectCoralL1(),
                frc2::cmd::Wait( 200_ms ),
                intake->StopCmd()
            ),
            [intake] {return intake->isCenterBroken();}
        ),
        MoveMechanism( arm, elevator, elevator::kElevatorMinHeight, arm::kCoralHoldPos, ArmIO::WristHorizontal ).ToPtr()
    );
}

frc2::CommandPtr IntakeCommands::CoralStationPickup( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        MoveMechanism( arm, elevator, elevator::kHeightCoralStation, arm::kElbowCoralStation, ArmIO::WristHorizontal ).ToPtr(),
        intake->IntakeCoral(),
        MoveMechanism( arm, elevator, elevator::kElevatorMinHeight, arm::kCoralHoldPos, ArmIO::WristHorizontal ).ToPtr()
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
        intake->IntakeCoral(),
        frc2::cmd::Parallel(
            MoveMechanism( arm, elevator, elevator::kElevatorMinHeight, arm::kCoralHoldPos, ArmIO::WristHorizontal ).ToPtr(),
            frc2::cmd::Either(
                    // Has Coral
                intake->IndexCoral(),
                    // Doesn't Have Coral, eject if one is half in the intake
                intake->EjectCoralL1(),
                [intake] {return intake->isCenterBroken();}
            )
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
