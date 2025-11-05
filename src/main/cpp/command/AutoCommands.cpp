
#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "command/AutoCommands.h"
#include "command/IntakeCommands.h"
#include "command/DriveCommands.h"
#include "command/DriveToPose.h"
#include "command/MoveMechanism.h"
#include "command/ControllerIO.h"

#include "arm/Arm.h"
#include "swerve/Drive.h"
#include "intake/Intake.h"
#include "elevator/Elevator.h"

#include "util/DataLogger.h"

#include "Constants.h"

using namespace physical;

frc2::CommandPtr AutoCommands::WaitForHoming( Arm *arm, Elevator *elevator )
{
    return frc2::cmd::WaitUntil( [arm,elevator] { return arm->isHomingDone() && elevator->isHomingDone(); } ).WithTimeout( 1_s );
}

frc2::CommandPtr AutoCommands::PrepareToPlaceOnReef( Arm *arm, Elevator *elevator, ReefPlacement p )
{
    return frc2::cmd::Sequence( 
        frc2::cmd::RunOnce( [] { DataLogger::Log( "AutoCommands/PrepareToPlaceOnReef", "Starting waiting for homing."); } ),
        WaitForHoming( arm, elevator ),
        frc2::cmd::RunOnce( [] { DataLogger::Log( "AutoCommands/PrepareToPlaceOnReef", "Done waiting for homing."); } ),
        frc2::cmd::Select<ReefPlacement>( 
            [p] { return p; }, 
            std::pair{ ReefPlacement::PLACING_L1, 
                MoveMechanism( arm, elevator, elevator::kHeightCoralL1, arm::kElbowCoralL1, ArmIO::WristHorizontal ).ToPtr() },
            std::pair{ ReefPlacement::PLACING_L2, 
                MoveMechanism( arm, elevator, elevator::kHeightCoralL2 + 4_in, arm::kElbowCoralL2 - 14_deg, ArmIO::WristVertical ).ToPtr() },
            std::pair{ ReefPlacement::PLACING_L3, 
                MoveMechanism( arm, elevator, elevator::kHeightCoralL3 + 4_in, arm::kElbowCoralL3 - 14_deg, ArmIO::WristVertical ).ToPtr() },
            std::pair{ ReefPlacement::PLACING_L4, 
                MoveMechanism( arm, elevator, elevator::kHeightCoralL4, arm::kElbowCoralL4, ArmIO::WristVertical ).ToPtr() }
        )
    );
}

frc2::CommandPtr AutoCommands::ReefToCoralStation( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            frc2::cmd::Wait(0.25_s).AndThen( frc2::cmd::RunOnce( [intake] { intake->Stop(); }, {intake}) ),
            MoveMechanism( arm, elevator, elevator::kHeightCoralStation, arm::kElbowCoralStation, ArmIO::WristHorizontal ).ToPtr()
        ),
        intake->IntakeCoralNoIndex( 10_s )
        // frc2::cmd::Parallel(
        //     frc2::cmd::Wait(0.25_s).AndThen( frc2::cmd::RunOnce( [intake] { intake->Stop(); }, {intake}) ),
        //     arm->ChangeElbowAndWrist( arm::kElbowForwardRaiseAngle, ArmIO::WristHorizontal )
        // ),
        // elevator->ChangeHeight( elevator::kElevatorMinHeight ),
        // arm->ChangeElbowAngle( arm::kElbowBackwardRaiseAngle ),
        // frc2::cmd::Parallel(
        //     elevator->ChangeHeight( elevator::kHeightCoralStation ),
        //     frc2::cmd::WaitUntil( [elevator] { return elevator->GetHeight() > 2_in;} )
        //         .AndThen( arm->ChangeElbowAngle( arm::kElbowCoralStation ) ),
        //     intake->IntakeCoralNoIndex( 10_s )
        // )
    ).WithName( "ReefToCoralStation" );
}

frc2::CommandPtr AutoCommands::CoralStationPickup( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            MoveMechanism( arm, elevator, elevator::kHeightCoralStation, arm::kElbowCoralStation, ArmIO::WristHorizontal ).ToPtr(),
            // elevator->ChangeHeight( elevator::kHeightCoralStation ),
            // frc2::cmd::WaitUntil( [elevator] { return elevator->GetHeight() > 2_in;} )
            //     .AndThen( arm->ChangeElbowAngle( arm::kElbowCoralStation ) ),
            intake->IntakeCoralNoIndex( 1.5_s )
        )
        // intake->IndexCoral()
    ).WithName( "CoralStationPickup" );
}

frc2::CommandPtr AutoCommands::LeaveCoralStation( Arm *arm, Intake *intake, Elevator *elevator, ReefPlacement p )
{
    return frc2::cmd::Parallel(
        intake->IndexCoral(),
        PrepareToPlaceOnReef( arm, elevator, p )
        // frc2::cmd::Sequence(
        //     arm->ChangeElbowAndWrist( arm::kElbowBackwardRaiseAngle, ArmIO::WristHorizontal ).WithTimeout( 0.5_s ),
        //     elevator->ChangeHeight( elevator::kElevatorMinHeight ),
        //     arm->ChangeElbowAngle( arm::kElbowForwardRaiseAngle ),
        //     frc2::cmd::Select<ReefPlacement>( 
        //         [p] { return p; }, 
        //         std::pair{ ReefPlacement::PLACING_L1, frc2::cmd::Sequence(
        //             elevator->ChangeHeight( elevator::kHeightCoralL1 ),
        //             arm->ChangeElbowAngle( arm::kElbowCoralL1 )
        //         )},
        //         std::pair{ ReefPlacement::PLACING_L2, elevator->ChangeHeight( elevator::kHeightCoralL2 + 4_in ) },
        //         std::pair{ ReefPlacement::PLACING_L3, elevator->ChangeHeight( elevator::kHeightCoralL3 + 4_in ) },
        //         std::pair{ ReefPlacement::PLACING_L4, elevator->ChangeHeight( elevator::kHeightCoralL3 ) }
        //     )
        // )
    ).WithName( "LeaveCoralStation" );
}

frc2::CommandPtr AutoCommands::AutoPlaceCoralL1( Intake *intake )
{
    return frc2::cmd::Sequence(
        // arm->ChangeElbowAndWrist( arm::kElbowCoralL1, ArmIO::WristHorizontal ),
        // DriveCommands::DriveDeltaPose( d, {reef::place_L1_shift_in, 0_in, 0_deg}, true, 1.0 ).WithTimeout(1_s),
        intake->EjectCoralL1()
    ).WithName("AutoPlaceCoralL1");
}

frc2::CommandPtr AutoCommands::AutoPlaceCoralL2( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        // frc2::cmd::Parallel(
        //     elevator->ChangeHeight( elevator::kHeightCoralL2 + 4_in ),
        //     arm->ChangeElbowAndWrist( arm::kElbowCoralL2 - 14_deg, ArmIO::WristVertical )
        // ),
        // ReefCommands::DriveToReefPoseDelta( d, {reef::place_L2_shift_in, 0_in, 0_deg}, onRightSide ).WithTimeout(1_s),
        frc2::cmd::Parallel(
            elevator->ChangeHeight( elevator::kHeightCoralL2 ),
            arm->ChangeElbowAngle( arm::kElbowCoralL2 )
        ),
        intake->EjectCoralL2_4_Fast(),
        frc2::cmd::Parallel(
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            arm->ChangeElbowAngle( arm::kElbowCoralL2 - 5_deg ).WithTimeout(0.1_s),
            elevator->ChangeHeight( elevator::kHeightCoralL2 - 3_in )
        )
    ).WithName("AutoPlaceCoralL2");
}

frc2::CommandPtr AutoCommands::AutoPlaceCoralL3( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        // frc2::cmd::Parallel(
        //     elevator->ChangeHeight( elevator::kHeightCoralL3 + 4_in ),
        //     arm->ChangeElbowAndWrist( arm::kElbowCoralL3 - 14_deg, ArmIO::WristVertical )
        // ),
        // ReefCommands::DriveToReefPoseDelta( d, {reef::place_L3_shift_in, 0_in, 0_deg}, onRightSide ).WithTimeout( 1_s),
        frc2::cmd::Parallel(
            elevator->ChangeHeight( elevator::kHeightCoralL3 ),
            arm->ChangeElbowAngle( arm::kElbowCoralL3 )
        ),
        intake->EjectCoralL2_4_Fast(),
        frc2::cmd::Parallel(
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            arm->ChangeElbowAngle( arm::kElbowCoralL3 - 5_deg ).WithTimeout(0.1_s),
            elevator->ChangeHeight( elevator::kHeightCoralL3 - 3_in )
        )
    ).WithName("AutoPlaceCoralL3");
}

frc2::CommandPtr AutoCommands::AutoPlaceCoralL4( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        // frc2::cmd::Parallel(
        //     arm->ChangeElbowAndWrist( arm::kElbowCoralL4, ArmIO::WristVertical ),
        //     elevator->ChangeHeight( elevator::kHeightCoralL4 )
        // ),
        // ReefCommands::DriveToReefPoseDelta( d, {reef::place_L4_shift_in, 0_in, 0_deg}, onRightSide ).WithTimeout( 1_s),
        intake->EjectCoralL2_4_Fast(),
        frc2::cmd::Parallel(
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            arm->ChangeElbowAngle( arm::kElbowCoralL4 - 10_deg ).WithTimeout(0.1_s),
            elevator->ChangeHeight( elevator::kHeightCoralL4 - 2_in )
        )
    ).WithName("AutoPlaceCoralL4");
}

frc2::CommandPtr AutoCommands::AutoEndAtReef( Drive *d, Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            frc2::cmd::Wait(0.5_s).AndThen( frc2::cmd::RunOnce( [intake] { intake->Stop(); }, {intake}) ),
            DriveCommands::DriveDeltaPose( d, {-15_in, 0_in, 0_deg}, true, 1.0 )
        ),
        IntakeCommands::RestPosition( arm, intake, elevator )
    ).WithName("AutoEndAtReef");
}

frc2::CommandPtr AutoCommands::AutoRestPosition( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        MoveMechanism( arm, elevator, elevator::kElevatorMinHeight, arm::kElbowRestAngle, ArmIO::WristHorizontal ).ToPtr(),
        frc2::cmd::RunOnce( [intake] {intake->Stop();}, {intake} )
    );
}
