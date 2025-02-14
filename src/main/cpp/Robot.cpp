// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

    // Global variables of mech2d
frc::Mechanism2d robot_mech{ 1.5, 2.5 };  // units are meters
frc::MechanismLigament2d* elevator_lig;
frc::MechanismLigament2d* elbow_lig;
frc::MechanismLigament2d* wrist_lig1;
frc::MechanismLigament2d* wrist_lig2;
frc::MechanismLigament2d* climber_lig;

Robot::Robot() 
{
    frc::MechanismRoot2d* elev_root = robot_mech.GetRoot("elev_root", 24/39.0, 5/39.0);
    elevator_lig = elev_root->Append<frc::MechanismLigament2d>("elevator", 9/39.0, 84_deg);
    elbow_lig = elevator_lig->Append<frc::MechanismLigament2d>("elbow", 15/39.0, 186_deg, 6, frc::Color8Bit{frc::Color::kGreen});
    frc::MechanismLigament2d* wrist_link = elbow_lig->Append<frc::MechanismLigament2d>("wristlink", 2/39.0, 0_deg, 6, frc::Color8Bit{frc::Color::kPurple});
    wrist_lig1 = wrist_link->Append<frc::MechanismLigament2d>("wrist1", 2/39.0, 90_deg, 10, frc::Color8Bit{frc::Color::kPurple});
    wrist_lig2 = wrist_link->Append<frc::MechanismLigament2d>("wrist2", 2/39.0, 270_deg, 10, frc::Color8Bit{frc::Color::kPurple});

    frc::MechanismRoot2d* climber_root = robot_mech.GetRoot("climber_root", 42/39.0, 6/39.0);
    climber_lig = climber_root->Append<frc::MechanismLigament2d>("climber", 11/39.0, 165_deg, 6, frc::Color8Bit{frc::Color::kRed});

    frc::SmartDashboard::PutData( "Robot/Mechanism2d", &robot_mech );
}

void Robot::RobotPeriodic() {
    LoggedRobot::RobotPeriodic();
    frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    m_autonomousCommand = m_container.GetAutonomousCommand();

    if (m_autonomousCommand) {
        m_autonomousCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
    if (m_autonomousCommand) {
        m_autonomousCommand->Cancel();
    }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
