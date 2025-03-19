
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "command/IntakeCommands.h"

#include "arm/Arm.h"
#include "swerve/Drive.h"
#include "intake/Intake.h"
#include "elevator/Elevator.h"

#include "Constants.h"

using namespace physical;

frc2::CommandPtr IntakeCommands::RestPosition( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Either(
        // Arm is in backward position and elevator is up
        frc2::cmd::Sequence(
            // arm->ChangeWristPosition( ArmIO::WristHorizontal ),
            // arm->ChangeElbowAngle( arm::kElbowBackwardRaiseAngle ),
            arm->ChangeElbowAndWrist( arm::kElbowBackwardRaiseAngle, ArmIO::WristHorizontal ),
            frc2::cmd::Parallel(
                frc2::cmd::RunOnce( [intake] {intake->Stop();}, {intake} ),
                elevator->ChangeHeight( 0.0_in )
            ),
            arm->ChangeElbowAngle( arm::kElbowRestAngle )
        ),
        // Arm is in forward position
        frc2::cmd::Sequence(
            // arm->ChangeWristPosition( ArmIO::WristHorizontal ),
            // arm->ChangeElbowAngle( arm::kElbowForwardRaiseAngle ),
            arm->ChangeElbowAndWrist( arm::kElbowForwardRaiseAngle, ArmIO::WristHorizontal ),
            frc2::cmd::Parallel(
                frc2::cmd::RunOnce( [intake] {intake->Stop();}, {intake} ),
                elevator->ChangeHeight( 0.0_in )
            ),
            arm->ChangeElbowAngle( arm::kElbowRestAngle )
        ),
        [arm, elevator] { return arm->isArmBackward(); }
    ).WithName( "Rest Position" );

}

frc2::CommandPtr IntakeCommands::CoralStationPickup( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        arm->ChangeWristPosition( ArmIO::WristHorizontal ),  
        elevator->ChangeHeight( elevator::kElevatorMinHeight ),
        arm->ChangeElbowAngle( arm::kElbowBackwardRaiseAngle ),
        elevator->ChangeHeight( elevator::kHeightCoralStation ),
        arm->ChangeElbowAngle( arm::kElbowCoralStation ),
        intake->IntakeCoral(),
        RestPosition( arm, intake, elevator )
    ).WithName( "Coral Station Pickup" );
}

frc2::CommandPtr IntakeCommands::CoralStationResume( Arm *arm, Intake *intake, Elevator *elevator, bool eject )
{
    return frc2::cmd::Sequence(
        elevator->ChangeHeight( elevator::kHeightCoralStation ),
        arm->ChangeElbowAngle( arm::kElbowCoralStation ),
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
        arm->ChangeWristPosition( ArmIO::WristHorizontal ),  
        frc2::cmd::Parallel(
            elevator->ChangeHeight( elevator::kHeightGroundPickup ),
            arm->ChangeElbowAngle( arm::kElbowGroundPickup )
        ),
        intake->IntakeCoral(),
        RestPosition( arm, intake, elevator )
    ).WithName( "Ground Pickup" );
}

frc2::CommandPtr IntakeCommands::GroundResume( Arm *arm, Intake *intake, Elevator *elevator, bool eject )
{
    return frc2::cmd::Sequence(
        elevator->ChangeHeight( elevator::kHeightGroundPickup ),
        arm->ChangeElbowAngle( arm::kElbowGroundPickup ),
        frc2::cmd::Either( 
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            intake->IntakeCoral(),
            [eject] { return eject; }
        )
    ).WithName( "Ground Resume" );
}

frc2::CommandPtr IntakeCommands::ElevatorRaise( Arm *arm, Elevator *elevator )
{
    return frc2::cmd::Sequence( 
        arm->ChangeElbowAngle( arm::kElbowForwardRaiseAngle ),
        elevator->ChangeHeight( elevator::kHeightDislogAlgae )
    );
}       
