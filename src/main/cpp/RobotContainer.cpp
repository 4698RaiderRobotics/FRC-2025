// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>

#include "swerve/Drive.h"
#include "intake/Intake.h"
#include "elevator/Elevator.h"

#include "command/DriveCommands.h"

RobotContainer::RobotContainer() {

    m_drive = new Drive( );
    m_intake = new Intake( );
    m_elevator = new Elevator( );

    ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
    m_drive->SetDefaultCommand( 
        DriveCommands::JoystickDrive( 
            m_drive,
            [this] { return -m_controller.GetLeftY(); },
            [this] { return -m_controller.GetLeftX(); },
            [this] { return m_controller.GetRightX(); }
        )
    );

    m_elevator->SetDefaultCommand(
        frc2::cmd::Run( [this] {
            if(m_controller.GetHID().GetLeftBumperButton()) {
              m_elevator->Nudge(elevator_axis.GetAxis() * 0.1_in);
            }
        },
        { m_elevator }
    ).WithName("Elevator Nudge"));


    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    (m_controller.Back() && m_controller.Y()).WhileTrue(m_drive->SysIdDynamic(frc2::sysid::Direction::kForward));
    (m_controller.Back() && m_controller.X()).WhileTrue(m_drive->SysIdDynamic(frc2::sysid::Direction::kReverse));
    (m_controller.Start() && m_controller.Y()).WhileTrue(m_drive->SysIdQuasistatic(frc2::sysid::Direction::kForward));
    (m_controller.Start() && m_controller.X()).WhileTrue(m_drive->SysIdQuasistatic(frc2::sysid::Direction::kReverse));

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    return frc2::cmd::Print("No autonomous command configured");
}
