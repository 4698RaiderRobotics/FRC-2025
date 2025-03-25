// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/DriverStation.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "util/EncOffsets.h"

    // Global variables of mech2d
frc::MechanismLigament2d* elevator_lig;
frc::MechanismLigament2d* elbow_lig;
frc::MechanismLigament2d* wrist_lig1;
frc::MechanismLigament2d* wrist_lig2;
frc::MechanismLigament2d* climber_lig;

Robot::Robot() 
{
    frc::MechanismRoot2d* elev_root = robot_mech.GetRoot("elev_root", 38/39.0, 5/39.0);
    elevator_lig = elev_root->Append<frc::MechanismLigament2d>("elevator", 9/39.0, 96_deg);
    elbow_lig = elevator_lig->Append<frc::MechanismLigament2d>("elbow", 15/39.0, -96_deg, 6, frc::Color8Bit{frc::Color::kGreen});
    frc::MechanismLigament2d* wrist_link = elbow_lig->Append<frc::MechanismLigament2d>("wristlink", 2/39.0, 0_deg, 6, frc::Color8Bit{frc::Color::kPurple});
    wrist_lig1 = wrist_link->Append<frc::MechanismLigament2d>("wrist1", 2/39.0, 90_deg, 10, frc::Color8Bit{frc::Color::kPurple});
    wrist_lig2 = wrist_link->Append<frc::MechanismLigament2d>("wrist2", 2/39.0, 270_deg, 10, frc::Color8Bit{frc::Color::kPurple});

    frc::MechanismRoot2d* climber_root = robot_mech.GetRoot("climber_root", 14/39.0, 6/39.0);
    climber_lig = climber_root->Append<frc::MechanismLigament2d>("climber", 11/39.0, 15_deg, 6, frc::Color8Bit{frc::Color::kRed});
}

void Robot::RobotInit() 
{
    // This not working in Robot::Robot() constructor ??
    frc::SmartDashboard::PutData( "Robot/Mechanism2d", &robot_mech );
    EncOffsets::GetInstance().SetupUI();
}

void Robot::RobotPeriodic() 
{
    LoggedRobot::RobotPeriodic();
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledPeriodic() 
{
    EncOffsets::GetInstance().UpdateUI();
}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() 
{
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand != nullptr) {
        fmt::print( " Scheduling Autonomous Command {} ....\n", m_autonomousCommand->GetName() );
        m_autonomousCommand->Schedule();
    } else {
        fmt::print( " Autonomous Command is NULL !!!!!!!!!!!!!!!!!\n" );
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() 
{
    if (m_autonomousCommand != nullptr) {
        m_autonomousCommand->Cancel();
    }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}


#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
