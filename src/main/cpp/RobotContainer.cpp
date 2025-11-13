// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Controls.h"
#include "RobotContainer.h"

#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>
#include <frc2/command/ScheduleCommand.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "arm/Arm.h"
#include "swerve/Drive.h"
#include "intake/Intake.h"
#include "climber/Climber.h"
#include "elevator/Elevator.h"
#include "vision/Vision.h"

#include "command/DriveCommands.h"
#include "command/ReefCommands.h"
#include "command/AutoCommands.h"
#include "command/IntakeCommands.h"
#include "command/ControllerIO.h"
#include "command/CoralViz.h"

ReefPlacement RobotContainer::next_reef_place = ReefPlacement::NONE;

RobotContainer::RobotContainer()
    : elevator_nudge_axis{ operatorCtrlr.GetHID(), ctrl::nudge_elevator_axis, true },
    elbow_nudge_axis{ operatorCtrlr.GetHID(), ctrl::nudge_elbow_axis, true },
    climber_nudge_axis{ operatorCtrlr.GetHID(), ctrl::nudge_climber_axis, true },
    nudge_hold_button{ operatorCtrlr.Button( ctrl::nudge_modifier ) }
{

    m_arm = new Arm( );
    m_drive = new Drive( );
    m_intake = new Intake( );
    m_climber = new Climber( );
    m_elevator = new Elevator( );
    m_vision = new Vision( &m_drive->m_odometry );

    ControllerIO::SetupControllerIO( m_drive, &driverCtrlr, &operatorCtrlr );

    // Extra deadband for the climber and elevator nudge
    climber_nudge_axis.SetDeadband( 0.25 );
    elevator_nudge_axis.SetDeadband( 0.25 );

    ConfigureDefaults();
    ConfigureBindings();
    ConfigureAutos();
    frc::SmartDashboard::PutData("Auto Mode", &m_chooser);
    LogReefPlacement( next_reef_place );
}

void RobotContainer::ConfigureDefaults() 
{
    m_drive->SetDefaultCommand( 
        DriveCommands::JoystickDrive( 
            m_drive,
            [this] { return -driverCtrlr.GetHID().GetRawAxis( ctrl::drive_X_axis ); },
            [this] { return -driverCtrlr.GetHID().GetRawAxis( ctrl::drive_Y_axis ); },
            [this] { return -driverCtrlr.GetHID().GetRawAxis( ctrl::drive_theta_axis ); }
        )
    );

    m_arm->SetDefaultCommand(
        frc2::cmd::Run( [this] {
            if( nudge_hold_button.Get() ) {
                m_arm->NudgeElbow(elbow_nudge_axis.GetAxis() * 0.5_deg);
                if( operatorCtrlr.GetHID().GetRawAxis(ctrl::nudge_wrist_axis) > 0.75 ) {
                    m_arm->SetWristGoal( ArmIO::WristHorizontal);
                } else if( operatorCtrlr.GetHID().GetRawAxis(ctrl::nudge_wrist_axis) < -0.75 ) {
                    m_arm->SetWristGoal( ArmIO::WristVertical);
                }
            }
        },
        { m_arm }
    ).WithName("Elbow-Wrist Nudge"));

    m_climber->SetDefaultCommand(
        frc2::cmd::Run( [this] {
            if( nudge_hold_button.Get() ) {
                m_climber->Nudge(climber_nudge_axis.GetAxis() * -0.5_deg);
            }
        },
        { m_climber }
    ).WithName("Climber Nudge"));

    m_elevator->SetDefaultCommand(
        frc2::cmd::Run( [this] {
            if( nudge_hold_button.Get() ) {
                m_elevator->Nudge(elevator_nudge_axis.GetAxis() * 0.25_in);
            }
        },
        { m_elevator }
    ).WithName("Elevator Nudge"));
}

void RobotContainer::ConfigureBindings() 
{
    /**************************          DRIVER           ********************* */
    (driverCtrlr.LeftBumper() && driverCtrlr.RightBumper()).Debounce( 100_ms )
        .OnTrue( frc2::cmd::RunOnce( [this] { m_drive->SetPose( {0_m, 0_m, 0_deg} ); }, {m_drive} ));

    driverCtrlr.Button( ctrl::cancel_button ).OnTrue(
        ReefCommands::CancelAll( m_arm, m_intake, m_elevator, m_climber )
    );

    driverCtrlr.Button( ctrl::manual_spin_down ).OnTrue( m_intake->EjectCoralL2_4_Fast() );

    driverCtrlr.Button( ctrl::manual_eject ).OnTrue( m_intake->EjectCoralL1() );

    // driverCtrlr.LeftStick().OnTrue( ReefCommands::DeployClimberFoot( m_arm, m_climber ) );
    driverCtrlr.Button( ctrl::raise_climber ).OnTrue( ReefCommands::LockClimberToCage( m_arm, m_climber ) );
    driverCtrlr.Button( ctrl::start_climb ).OnTrue( m_climber->DoClimb() );

    driverCtrlr.AxisGreaterThan( ctrl::manual_index, 0.75).OnTrue(
        m_intake->IndexCoral()
    );

    driverCtrlr.Button( ctrl::cancel_intake).OnTrue( m_intake->StopIndex() );

    driverCtrlr.RightStick().OnTrue(IntakeCommands::ManualPlace2(m_arm, m_intake, m_elevator));
    driverCtrlr.LeftStick().OnTrue(IntakeCommands::ManualPlace3(m_arm, m_intake, m_elevator));

    // driverCtrlr.POV( 0 ).OnTrue( DriveToPose( m_drive, [] { return frc::Pose2d{ 610_in, 158.50_in, 180_deg}; } ).ToPtr() );

    /**************************          OPERATOR           ********************* */
    operatorCtrlr.Button( ctrl::cancel_button ).OnTrue( 
        ReefCommands::CancelAll( m_arm, m_intake, m_elevator, m_climber )
    );

    (!nudge_hold_button && operatorCtrlr.Button( ctrl::pick_L1_level ))
        .OnTrue( SetReefPlacement( m_arm, m_elevator, m_intake, ReefPlacement::PLACING_L1 ) );
    (!nudge_hold_button && operatorCtrlr.Button( ctrl::pick_L2_level ))
        .OnTrue( SetReefPlacement( m_arm, m_elevator, m_intake, ReefPlacement::PLACING_L2 ) );
    (!nudge_hold_button && operatorCtrlr.Button( ctrl::pick_L3_level ))
        .OnTrue( SetReefPlacement( m_arm, m_elevator, m_intake, ReefPlacement::PLACING_L3 ) );
    (!nudge_hold_button && operatorCtrlr.Button( ctrl::pick_L4_level ))
        .OnTrue( SetReefPlacement( m_arm, m_elevator, m_intake, ReefPlacement::PLACING_L4 ) );

    operatorCtrlr.AxisGreaterThan( ctrl::place_on_reef_left, 0.75 )
        .OnTrue( frc2::cmd::Sequence( 
            ReefCommands::PlaceOnReef( m_drive, m_arm, m_intake, m_elevator, false, [] { return next_reef_place; } ),
            SetReefPlacement( m_arm, m_elevator, m_intake, ReefPlacement::NONE )
        ));
    operatorCtrlr.AxisGreaterThan( ctrl::place_on_reef_right, 0.75 )
        .OnTrue( frc2::cmd::Sequence( 
            ReefCommands::PlaceOnReef( m_drive, m_arm, m_intake, m_elevator, true, [] { return next_reef_place; } ),
            SetReefPlacement( m_arm, m_elevator, m_intake, ReefPlacement::NONE )
        ));

    (operatorCtrlr.POV( ctrl::intake_ground ) || operatorCtrlr.POV( ctrl::intake_ground + 45 ) || operatorCtrlr.POV( ctrl::intake_ground - 45 ) && !operatorCtrlr.Start())
        .OnTrue( IntakeCommands::GroundPickup( m_arm, m_intake, m_elevator ) )
        .OnFalse( IntakeCommands::CoralHoldPos( m_arm, m_intake, m_elevator ) );

    ((operatorCtrlr.POV( ctrl::intake_coral_station ) || operatorCtrlr.POV( ctrl::intake_coral_station + 45 ) || operatorCtrlr.POV( 315 )) && !operatorCtrlr.Start())
        .OnTrue( IntakeCommands::CoralStationPickup( m_arm, m_intake, m_elevator ) )
        .OnFalse( IntakeCommands::CoralHoldPos( m_arm, m_intake, m_elevator ) );

    (operatorCtrlr.POV( ctrl::intake_ground ) && operatorCtrlr.Start())
        .WhileTrue( IntakeCommands::GroundResume( m_arm, m_intake, m_elevator, true ))
        .OnFalse(IntakeCommands::GroundResume( m_arm, m_intake, m_elevator, false ));

    (operatorCtrlr.POV( ctrl::intake_coral_station ) && operatorCtrlr.Start())
        .WhileTrue( IntakeCommands::CoralStationResume( m_arm, m_intake, m_elevator, true ))
        .OnFalse(IntakeCommands::CoralStationResume( m_arm, m_intake, m_elevator, false ));

    operatorCtrlr.RightStick().OnTrue( ReefCommands::RemoveAlgae( m_drive, m_arm, m_intake, m_elevator ));

    operatorCtrlr.LeftStick().OnTrue( IntakeCommands::ElevatorRaise( m_arm, m_elevator ));


    /**************************          TRIGGERS          ********************* */

    m_intake->HasCoralTrigger().OnTrue( 
        frc2::cmd::Parallel(
            CoralViz( [this] { return m_drive->GetPose(); }, [this] {return m_intake->isCenterBroken();} ).ToPtr(),
            ControllerIO::CoralRumble()
        )
    );

    // m_climber->isHoming()
    //     .OnTrue( m_arm->SetClimberHoming( true )) 
    //     .OnFalse( m_arm->SetClimberHoming( false ));


    /**************************          DEBUG MODE          ********************* */
    if( !frc::DriverStation::IsFMSAttached() ) {
        // (nudge_hold_button && operatorCtrlr.Button( ctrl::manual_intake_algae ))
        //     .OnTrue( m_intake->IntakeAlgae() );
        // (nudge_hold_button && operatorCtrlr.Button( ctrl::manual_intake_coral ))
        //     .OnTrue( m_intake->IntakeCoral() );
        // (nudge_hold_button && operatorCtrlr.Button( ctrl::manual_eject ))
        //     .OnTrue( m_intake->EjectCoralL1() );
        // (nudge_hold_button && operatorCtrlr.Button( ctrl::manual_spin_down ))
        //     .OnTrue( m_intake->EjectCoralL2_4( false ) );
    (nudge_hold_button && operatorCtrlr.A()).WhileTrue( frc2::cmd::Run( [this] {m_intake->SpinIn();}, {m_intake} ))
        .OnFalse(frc2::cmd::RunOnce( [this] {m_intake->Stop();}, {m_intake} ));
    (nudge_hold_button && operatorCtrlr.Y()).WhileTrue( frc2::cmd::Run( [this] {m_intake->SpinOut();}, {m_intake} ))
        .OnFalse(frc2::cmd::RunOnce( [this] {m_intake->Stop();}, {m_intake} ));
    // (nudge_hold_button && operatorCtrlr.X()).WhileTrue( frc2::cmd::Run( [this] {m_intake->ShiftDown();}, {m_intake} ))
    //     .OnFalse(frc2::cmd::RunOnce( [this] {m_intake->Stop();}, {m_intake} ));
    (nudge_hold_button && operatorCtrlr.X()).WhileTrue( m_intake->IntakeCoralNoIndex( 10_s ) )
        .OnFalse( m_intake->IndexCoral() );

    }

    /**************************          SYSID          ********************* */
    // if( !frc::DriverStation::IsFMSAttached() ) {
    //     // Run SysId routines when holding back/start and X/Y.
    //     // Note that each routine should be run exactly once in a single log.
    //     (driverCtrlr.Back() && driverCtrlr.Y()).WhileTrue(m_drive->SysIdDynamic(frc2::sysid::Direction::kForward));
    //     (driverCtrlr.Back() && driverCtrlr.X()).WhileTrue(m_drive->SysIdDynamic(frc2::sysid::Direction::kReverse));
    //     (driverCtrlr.Start() && driverCtrlr.Y()).WhileTrue(m_drive->SysIdQuasistatic(frc2::sysid::Direction::kForward));
    //     (driverCtrlr.Start() && driverCtrlr.X()).WhileTrue(m_drive->SysIdQuasistatic(frc2::sysid::Direction::kReverse));
    // }

}

void RobotContainer::ConfigureAutos()
{
    pathplanner::NamedCommands::registerCommand(
        "RestPosition", 
        AutoCommands::AutoRestPosition( m_arm, m_intake, m_elevator )
    );
    pathplanner::NamedCommands::registerCommand(
        "PlaceOnReefL1", 
        AutoCommands::AutoPlaceCoralL1( m_intake )
    );
    pathplanner::NamedCommands::registerCommand(
        "PlaceOnReefL2", 
        AutoCommands::AutoPlaceCoralL2( m_arm, m_intake, m_elevator )
    );
    pathplanner::NamedCommands::registerCommand(
        "PlaceOnReefL3", 
        AutoCommands::AutoPlaceCoralL3( m_arm, m_intake, m_elevator )
    );
    pathplanner::NamedCommands::registerCommand(
        "PlaceOnReefL4", 
        AutoCommands::AutoPlaceCoralL4( m_arm, m_intake, m_elevator )
    );
    pathplanner::NamedCommands::registerCommand(
        "ReefToCoralStation",
        AutoCommands::ReefToCoralStation( m_arm, m_intake, m_elevator )
    );
    pathplanner::NamedCommands::registerCommand(
        "PrepareToPlaceOnReefL1", 
        AutoCommands::PrepareToPlaceOnReef( m_arm, m_elevator, ReefPlacement::PLACING_L1 )
    );
    pathplanner::NamedCommands::registerCommand(
        "PrepareToPlaceOnReefL3", 
        AutoCommands::PrepareToPlaceOnReef( m_arm, m_elevator, ReefPlacement::PLACING_L3 )
    );
    pathplanner::NamedCommands::registerCommand(
        "PrepareToPlaceOnReefL4", 
        AutoCommands::PrepareToPlaceOnReef( m_arm, m_elevator, ReefPlacement::PLACING_L4 )
    );
    pathplanner::NamedCommands::registerCommand(
        "LeaveCoralStationToL3", 
        AutoCommands::LeaveCoralStation( m_arm, m_intake, m_elevator, ReefPlacement::PLACING_L3 )
    );
    pathplanner::NamedCommands::registerCommand(
        "LeaveCoralStationToL4", 
        AutoCommands::LeaveCoralStation( m_arm, m_intake, m_elevator, ReefPlacement::PLACING_L4 )
    );
    pathplanner::NamedCommands::registerCommand(
        "CoralStationPickup", 
        AutoCommands::CoralStationPickup( m_arm, m_intake, m_elevator )
    );
    pathplanner::NamedCommands::registerCommand(
        "AutoEndAtReef", 
        AutoCommands::AutoEndAtReef( m_drive, m_arm, m_intake, m_elevator )
    );

    std::vector<AutoNameMap> autos = { 
        // {"Center One Piece L1", "CenterOnePieceL1"},
        {"Center One Piece L4", "CenterOnePieceL4"},
        // {"Center to left source two piece", "CenterToLeftSideSourceTwoPiece"},
        // {"Center to right source two piece", "CenterToRightSideSourceTwoPiece"},
    
        // {"Left One Piece L1", "LeftOnePieceL1"},
        // {"Left One Piece L3", "LeftOnePieceL3"},
        // {"Left One Piece L4", "LeftOnePieceL4"},
        // {"Left Two Piece P55 L13", "LeftTwoPiece-P55-L13"},
        {"Left Two Piece P55 L14", "LeftTwoPiece-P55-L14"},
        // {"Left Three Piece L3", "LeftThreePieceL3"},
        {"Left Three Piece P655 L444", "LeftThreePiece-P655-L444"},
    
        // {"Right One Piece L1", "RightOnePieceL1"},
        // {"Right One Piece L4", "RightOnePieceL4"},
        // {"Right Two Piece L4", "RightTwoPieceL4"},
        // {"Right Two Piece P33 L13", "RightTwoPiece-P33-L13"},
        {"Right Two Piece P33 L14", "RightTwoPiece-P33-L14"},
        // {"Right Three Piece L3", "RightThreePieceL3"},
        {"Right Three Piece P233 L444", "RightThreePiece-P233-L444"},
        // {"Right Three Piece L4", "RightThreePieceL4"}
    };

    AutoCommands.push_back( DriveCommands::DriveDeltaPose( m_drive, {1.5_m, 0_m, 0_deg}, true, 0.5 ));
    m_chooser.SetDefaultOption( "Leave ONLY", 0 );

    for( unsigned int i=0; i<autos.size(); ++i ) {
        try {
            AutoCommands.push_back( pathplanner::PathPlannerAuto(autos[i].AutoName).WithName(autos[i].AutoName) );
            m_chooser.AddOption( autos[i].Description, i+1 );
        } catch(const std::exception& e) {
            fmt::print( "\n\n================> PathPlanner Auto NOT FOUND <==================\n" );
            fmt::print( "Attempted to load auto \"{}\".\n\n", autos[i].AutoName );
            fmt::print( "{}\n", e.what() );
            fmt::print( "====================================================================\n\n" );
            m_chooser.AddOption( fmt::format("<LOAD FAILED> {}", autos[i].Description), i );
        }
    }
}

frc2::Command* RobotContainer::GetAutonomousCommand() 
{
    return AutoCommands[ m_chooser.GetSelected() ].get();
}

frc2::CommandPtr RobotContainer::SetReefPlacement( Arm* arm, Elevator *elevator, Intake *intake, ReefPlacement p )
{
    return frc2::cmd::Sequence(
        frc2::cmd::RunOnce( [intake] {intake->enableRetention(false);}),
        frc2::cmd::RunOnce( [p] { next_reef_place = p; LogReefPlacement( p ); } ),
        frc2::cmd::Either(
            ReefCommands::PrepareToPlaceOnReef( arm, elevator, [p] { return p;} ),
            frc2::cmd::None(),
            [p] { return p != ReefPlacement::NONE;}
        )
    );
}

void RobotContainer::LogReefPlacement( ReefPlacement p )
{
    std::string logStr;
    
    switch( p ) {
    case ReefPlacement::NONE:
        logStr = "NONE";
        break;
    case ReefPlacement::PLACING_L1:
        logStr = "PLACING_L1";
        break;
    case ReefPlacement::PLACING_L2:
        logStr = "PLACING_L2";
        break;
    case ReefPlacement::PLACING_L3:
        logStr = "PLACING_L3";
        break;
    case ReefPlacement::PLACING_L4:
        logStr = "PLACING_L4";
        break;
    }

    DataLogger::Log( "ReefCommands/Reef Place", logStr );
}
