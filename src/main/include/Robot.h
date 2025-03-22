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
// extern frc::Mechanism2d robot_mech;
extern frc::MechanismLigament2d* elevator_lig;
extern frc::MechanismLigament2d* elbow_lig;
extern frc::MechanismLigament2d* wrist_lig1;
extern frc::MechanismLigament2d* wrist_lig2;
extern frc::MechanismLigament2d* climber_lig;

class Robot : public LoggedRobot {
public:
    Robot();
    void RobotInit() override;
    void RobotPeriodic() override;
    void DisabledPeriodic() override;
    void DisabledExit() override;
    void AutonomousInit() override;
    void AutonomousPeriodic() override;
    void AutonomousExit() override;
    void TeleopInit() override;
    void TeleopPeriodic() override;
    void TeleopExit() override;

private:
    frc2::Command* m_autonomousCommand{nullptr};

    RobotContainer m_container;

    frc::Mechanism2d robot_mech{ 60/39.0, 100/39.0 };  // units are meters
};
