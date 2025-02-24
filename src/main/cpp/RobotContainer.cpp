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
#include "command/IntakeCommands.h"
#include "command/DriveToPose.h"
#include "command/CoralViz.h"


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

    // Extra deadband for the climber and elevator nudge
    climber_nudge_axis.SetDeadband( 0.25 );
    elevator_nudge_axis.SetDeadband( 0.25 );

    ConfigureBindings();
    ConfigureAutos();
    frc::SmartDashboard::PutData("Auto Mode", &m_chooser);
    ReefCommands::SetReefPlacement( ReefPlacement::NONE );
}

void RobotContainer::ConfigureBindings() 
{
    m_arm->SetDefaultCommand(
        frc2::cmd::Run( [this] {
            if( nudge_hold_button.Get() ) {
                m_arm->NudgeElbow(elbow_nudge_axis.GetAxis() * 0.5_deg);
                if( operatorCtrlr.GetHID().GetRawAxis(ctrl::nudge_wrist_axis) > 0.75 ) {
                    m_arm->SetWristPosition( ArmIO::WristHorizontal);
                } else if( operatorCtrlr.GetHID().GetRawAxis(ctrl::nudge_wrist_axis) < -0.75 ) {
                    m_arm->SetWristPosition( ArmIO::WristVertical);
                }
            }
        },
        { m_arm }
    ).WithName("Elbow-Wrist Nudge"));

    m_drive->SetDefaultCommand( 
        DriveCommands::JoystickDrive( 
            m_drive,
            [this] { return -driverCtrlr.GetHID().GetRawAxis( ctrl::drive_X_axis ); },
            [this] { return -driverCtrlr.GetHID().GetRawAxis( ctrl::drive_Y_axis ); },
            [this] { return driverCtrlr.GetHID().GetRawAxis( ctrl::drive_theta_axis ); }
        )
    );

    m_climber->SetDefaultCommand(
        frc2::cmd::Run( [this] {
            if( nudge_hold_button.Get() ) {
                m_climber->Nudge(climber_nudge_axis.GetAxis() * -0.25_in);
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


    operatorCtrlr.A().OnTrue( frc2::cmd::RunOnce( [] { ReefCommands::SetReefPlacement( ReefPlacement::PLACING_L1 ); }) );
    operatorCtrlr.B().OnTrue( frc2::cmd::RunOnce( [] { ReefCommands::SetReefPlacement( ReefPlacement::PLACING_L2 ); }) );
    operatorCtrlr.Y().OnTrue( frc2::cmd::RunOnce( [] { ReefCommands::SetReefPlacement( ReefPlacement::PLACING_L3 ); }) );
    operatorCtrlr.RightBumper().OnTrue( frc2::cmd::RunOnce( [] { ReefCommands::SetReefPlacement( ReefPlacement::PLACING_L4 ); }) );
    operatorCtrlr.RightTrigger().OnTrue( ReefCommands::PlaceOnReef( m_drive, m_arm, m_intake, m_elevator, true ) );
    operatorCtrlr.LeftTrigger().OnTrue( ReefCommands::PlaceOnReef( m_drive, m_arm, m_intake, m_elevator, false ) );

    operatorCtrlr.POVDown()
        .WhileTrue( IntakeCommands::GroundPickup( m_arm, m_intake, m_elevator ) )
        .OnFalse( frc2::cmd::RunOnce( [] { /* Do Nothing. Just interrupt */ }, {m_arm, m_intake, m_elevator} ) );
    operatorCtrlr.POVUp().OnTrue( IntakeCommands::CoralStationPickup( m_arm, m_intake, m_elevator ) );

    m_intake->HasCoralTrigger().OnTrue( 
        frc2::cmd::Parallel(
            CoralViz( [this] { return m_drive->GetPose(); }, [this] {return m_intake->isCenterBroken();} ).ToPtr(),
            frc2::cmd::Sequence(
                frc2::cmd::RunOnce( [this] { operatorCtrlr.SetRumble( frc::GenericHID::kBothRumble, 0.9 ); } ),
                frc2::cmd::RunOnce( [this] { driverCtrlr.SetRumble( frc::GenericHID::kBothRumble, 0.9 ); } ),
                frc2::cmd::Wait( 0.5_s ),
                frc2::cmd::RunOnce( [this] { operatorCtrlr.SetRumble( frc::GenericHID::kBothRumble, 0 ); } ),
                frc2::cmd::RunOnce( [this] { driverCtrlr.SetRumble( frc::GenericHID::kBothRumble, 0 ); } )
            )
        )
    );


    if( !frc::DriverStation::IsFMSAttached() ) {
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        (driverCtrlr.Back() && driverCtrlr.Y()).WhileTrue(m_drive->SysIdDynamic(frc2::sysid::Direction::kForward));
        (driverCtrlr.Back() && driverCtrlr.X()).WhileTrue(m_drive->SysIdDynamic(frc2::sysid::Direction::kReverse));
        (driverCtrlr.Start() && driverCtrlr.Y()).WhileTrue(m_drive->SysIdQuasistatic(frc2::sysid::Direction::kForward));
        (driverCtrlr.Start() && driverCtrlr.X()).WhileTrue(m_drive->SysIdQuasistatic(frc2::sysid::Direction::kReverse));
    }

}

void RobotContainer::ConfigureAutos()
{
    pathplanner::NamedCommands::registerCommand("DriveToReefPoseLeft", ReefCommands::DriveToReefPose( m_drive, true ));
    pathplanner::NamedCommands::registerCommand("DriveToReefPoseRight", ReefCommands::DriveToReefPose( m_drive, false ));
  
    std::vector<AutoNameMap> autos = { 
        {"Center to left source two piece", "CenterToLeftSideSourceTwoPiece"},
        {"Center to right source two piece", "CenterToRightSideSourceTwoPiece"},
    
        {"Left One Piece", "LeftOnePiece"},
        {"Left Two Piece", "LeftTwoPiece"},
    
        {"Right One Piece", "RightOnePiece"},
        {"Right Two Piece", "RightTwoPiece"}
    };

    // std::vector<AutoNameMap> choreoAutos = { 
    //     {"Test Choreo", "TestPath"}
    // };

    for( unsigned int i=0; i<autos.size(); ++i ) {
        m_chooser.AddOption( autos[i].Description, i );
        AutoCommands.push_back( pathplanner::PathPlannerAuto(autos[i].AutoName).WithName(autos[i].AutoName) );
    }
    m_chooser.SetDefaultOption( autos[0].Description, 0 );

    // for( unsigned int i=0; i<choreoAutos.size(); ++i ) {
    //     m_chooser.AddOption( choreoAutos[i].Description, i + autos.size() );

    //     AutoCommands.push_back( 
    //         pathplanner::AutoBuilder::followPath( 
    //             pathplanner::PathPlannerPath::fromChoreoTrajectory(choreoAutos[i].AutoName)
    //         ).WithName(choreoAutos[i].AutoName) 
    //     );
    // }
}

frc2::Command* RobotContainer::GetAutonomousCommand() 
{
    return AutoCommands[ m_chooser.GetSelected() ].get();
}

void RobotContainer::UpdateElbowOffset()
{ 
    m_arm->UpdateElbowOffset(); 
}