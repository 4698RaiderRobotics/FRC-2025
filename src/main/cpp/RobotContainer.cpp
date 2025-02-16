// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Controls.h"
#include "RobotContainer.h"

#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "arm/Arm.h"
#include "swerve/Drive.h"
#include "intake/Intake.h"
#include "climber/Climber.h"
#include "elevator/Elevator.h"

#include "command/DriveCommands.h"
#include "command/DriveToPose.h"

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

    // Extra deadband for the climber and elevator nudge
    climber_nudge_axis.SetDeadband( 0.25 );
    elevator_nudge_axis.SetDeadband( 0.25 );

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
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
    ).WithName("Elbow/Wrist Nudge"));

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


    driverCtrlr.A().OnTrue( DriveToPose( m_drive, [] { return frc::Pose2d{ 5_m, 5_m, 0_deg }; } ).ToPtr() );
    driverCtrlr.B().OnTrue( DriveCommands::DriveDeltaPose( m_drive, frc::Transform2d{ 1_m, 1_m, 90_deg }, false ) );
    driverCtrlr.X().OnTrue( DriveCommands::DriveDeltaPose( m_drive, frc::Transform2d{ 1_m, 1_m, 90_deg }, true ) );



    if( !frc::DriverStation::IsFMSAttached() ) {
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        (driverCtrlr.Back() && driverCtrlr.Y()).WhileTrue(m_drive->SysIdDynamic(frc2::sysid::Direction::kForward));
        (driverCtrlr.Back() && driverCtrlr.X()).WhileTrue(m_drive->SysIdDynamic(frc2::sysid::Direction::kReverse));
        (driverCtrlr.Start() && driverCtrlr.Y()).WhileTrue(m_drive->SysIdQuasistatic(frc2::sysid::Direction::kForward));
        (driverCtrlr.Start() && driverCtrlr.X()).WhileTrue(m_drive->SysIdQuasistatic(frc2::sysid::Direction::kReverse));
    }

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    return frc2::cmd::Print("No autonomous command configured");
}
