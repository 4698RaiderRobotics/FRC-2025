// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/button/CommandXBoxController.h>
#include <frc2/command/CommandPtr.h>

#include "util/ControllerAxis.h"

class Drive;
class Intake;
class Elevator;

class RobotContainer {
public:
    RobotContainer();

    frc2::CommandPtr GetAutonomousCommand();

private:
    void ConfigureBindings();

    Drive *m_drive;
    Intake *m_intake;
    Elevator *m_elevator;

    frc2::CommandXboxController m_controller{0};
    ControllerAxis elevator_axis{ m_controller.GetHID(), frc::XboxController::Axis::kLeftX, true };

};
