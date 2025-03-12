
#include <frc/MathUtil.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "command/AutoCommands.h"
#include "command/IntakeCommands.h"
#include "command/DriveCommands.h"
#include "command/DriveToPose.h"
#include "command/ControllerIO.h"

#include "arm/Arm.h"
#include "swerve/Drive.h"
#include "intake/Intake.h"
#include "elevator/Elevator.h"

#include "Constants.h"

using namespace physical;


frc2::CommandPtr AutoCommands::PrepareToPlaceOnReef( Arm *arm, Elevator *elevator, ReefPlacement p )
{
    return frc2::cmd::Sequence(
        elevator->ChangeHeight( elevator::kElevatorMinHeight ),
        arm->ChangeElbowAngle( arm::kElbowForwardRaiseAngle ),
        frc2::cmd::Select<ReefPlacement>( 
            [p] { return p; }, 
            std::pair{ ReefPlacement::PLACING_L1, elevator->ChangeHeight( elevator::kHeightCoralL1 ) },
            std::pair{ ReefPlacement::PLACING_L2, elevator->ChangeHeight( elevator::kHeightCoralL2 ) },
            std::pair{ ReefPlacement::PLACING_L3, elevator->ChangeHeight( elevator::kHeightCoralL3 ) },
            std::pair{ ReefPlacement::PLACING_L4, elevator->ChangeHeight( elevator::kHeightCoralL3 ) }
        )
    );
}

frc2::CommandPtr AutoCommands::ReefToCoralStation( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        arm->ChangeElbowAndWrist( arm::kElbowForwardRaiseAngle, ArmIO::WristHorizontal ),
        elevator->ChangeHeight( elevator::kElevatorMinHeight ),
        arm->ChangeElbowAngle( arm::kElbowBackwardRaiseAngle ),
        elevator->ChangeHeight( elevator::kHeightCoralStation ),
        arm->ChangeElbowAngle( arm::kElbowCoralStation ),
        intake->IntakeCoralNoIndex()
    ).WithName( "PrepareForCoralStation" );
}

frc2::CommandPtr AutoCommands::CoralStationPickup( Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Sequence(
        intake->IntakeCoralNoIndex()
    ).WithName( "CoralStationPickup" );
}

frc2::CommandPtr AutoCommands::LeaveCoralStation( Arm *arm, Intake *intake, Elevator *elevator, ReefPlacement p )
{
    return frc2::cmd::Parallel(
        intake->IndexCoral(),
        frc2::cmd::Sequence(
            arm->ChangeElbowAndWrist( arm::kElbowBackwardRaiseAngle, ArmIO::WristHorizontal ),
            elevator->ChangeHeight( elevator::kElevatorMinHeight ),
            arm->ChangeElbowAngle( arm::kElbowForwardRaiseAngle ),
            frc2::cmd::Select<ReefPlacement>( 
                [p] { return p; }, 
                std::pair{ ReefPlacement::PLACING_L1, elevator->ChangeHeight( elevator::kHeightCoralL1 ) },
                std::pair{ ReefPlacement::PLACING_L2, elevator->ChangeHeight( elevator::kHeightCoralL2 ) },
                std::pair{ ReefPlacement::PLACING_L3, elevator->ChangeHeight( elevator::kHeightCoralL3 ) },
                std::pair{ ReefPlacement::PLACING_L4, elevator->ChangeHeight( elevator::kHeightCoralL3 ) }
            )
        )
    ).WithName( "LeaveCoralStation" );
}

frc2::CommandPtr AutoCommands::AutoPlaceCoralL1( Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide )
{
    return frc2::cmd::Sequence(
        arm->ChangeElbowAndWrist( arm::kElbowCoralL1, ArmIO::WristHorizontal ),
        DriveCommands::DriveDeltaPose( d, {reef_place_L1_shift_in, 0_in, 0_deg}, true, 0.5 ).WithTimeout( 1_s),
        intake->EjectCoralL1(),
        DriveCommands::DriveDeltaPose( d, {-8_in, 0_in, 0_deg}, true, 1.0 ),
        frc2::cmd::RunOnce( [intake] { intake->Stop(); })
    ).WithName("AutoPlaceCoralL1");
}

frc2::CommandPtr AutoCommands::AutoPlaceCoralL2( Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide )
{
    return frc2::cmd::Sequence(
        arm->ChangeElbowAndWrist( arm::kElbowCoralL2, ArmIO::WristVertical ),
        ReefCommands::DriveToReefPoseDelta( d, {reef_place_L2_shift_in, 0_in, 0_deg}, onRightSide ).WithTimeout( 1_s),
        intake->EjectCoralL2_4( false ),
        frc2::cmd::Parallel(
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            arm->ChangeElbowAngle( arm::kElbowCoralL2 - 10_deg ).WithTimeout(0.1_s),
            elevator->ChangeHeight( elevator::kHeightCoralL2 - 3_in )
        ),
        frc2::cmd::RunOnce( [intake] { intake->Stop(); }, {intake})
    ).WithName("AutoPlaceCoralL2");
}

frc2::CommandPtr AutoCommands::AutoPlaceCoralL3( Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide )
{
    return frc2::cmd::Sequence(
        arm->ChangeElbowAndWrist( arm::kElbowCoralL3, ArmIO::WristVertical ),
        ReefCommands::DriveToReefPoseDelta( d, {reef_place_L3_shift_in, 0_in, 0_deg}, onRightSide ).WithTimeout( 1_s),
        intake->EjectCoralL2_4( false ),
        frc2::cmd::Parallel(
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            arm->ChangeElbowAngle( arm::kElbowCoralL3 - 10_deg ).WithTimeout(0.1_s),
            elevator->ChangeHeight( elevator::kHeightCoralL3 - 3_in )
        ),
        frc2::cmd::RunOnce( [intake] { intake->Stop(); }, {intake})
    ).WithName("AutoPlaceCoralL3");
}

frc2::CommandPtr AutoCommands::AutoPlaceCoralL4( Drive *d, Arm *arm, Intake *intake, Elevator *elevator, bool onRightSide )
{
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            arm->ChangeElbowAndWrist( arm::kElbowCoralL4, ArmIO::WristVertical ),
            elevator->ChangeHeight( elevator::kHeightCoralL4 )
        ),
        ReefCommands::DriveToReefPoseDelta( d, {reef_place_L4_shift_in, 0_in, 0_deg}, onRightSide ).WithTimeout( 1_s),
        intake->EjectCoralL2_4( false ),
        frc2::cmd::Parallel(
            frc2::cmd::RunOnce( [intake] { intake->SpinOut(); }),
            arm->ChangeElbowAngle( arm::kElbowCoralL4 - 10_deg ).WithTimeout(0.1_s),
            elevator->ChangeHeight( elevator::kHeightCoralL4 - 3_in )
        ),
        frc2::cmd::RunOnce( [intake] { intake->Stop(); }, {intake})
    ).WithName("AutoPlaceCoralL4");
}

frc2::CommandPtr AutoCommands::AutoEndAtReef( Drive *d, Arm *arm, Intake *intake, Elevator *elevator )
{
    return frc2::cmd::Parallel(
        DriveCommands::DriveDeltaPose( d, {-15_in, 0_in, 0_deg}, true, 1.0 ),
        IntakeCommands::RestPosition( arm, intake, elevator )
    ).WithName("AutoEndAtReef");
}
