// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

#include "util/LoggedRobot.h"
#include "RobotContainer.h"

    // Global variables of mech2d
extern frc::Mechanism2d robot_mech;
extern frc::MechanismLigament2d* elevator_lig;
extern frc::MechanismLigament2d* elbow_lig;
extern frc::MechanismLigament2d* wrist_lig1;
extern frc::MechanismLigament2d* wrist_lig2;
extern frc::MechanismLigament2d* climber_lig;

class Robot : public LoggedRobot {
public:
    Robot();
    void RobotPeriodic() override;
    void DisabledInit() override;
    void DisabledPeriodic() override;
    void DisabledExit() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void AutonomousExit() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TeleopExit() override;
    void TestInit() override;
    void TestPeriodic() override;
    void TestExit() override;

private:
    std::optional<frc2::CommandPtr> m_autonomousCommand;

    RobotContainer m_container;
};
