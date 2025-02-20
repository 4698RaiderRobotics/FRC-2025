// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/button/CommandXBoxController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>

#include "util/ControllerAxis.h"

#include <frc/smartdashboard/SendableChooser.h>

class Arm;
class Drive;
class Intake;
class Climber;
class Elevator;
class Vision;

class RobotContainer {
public:
    RobotContainer();

    frc2::Command* GetAutonomousCommand();

private:
    void ConfigureBindings();
    void ConfigureAutos();

    Arm *m_arm;
    Drive *m_drive;
    Intake *m_intake;
    Climber *m_climber;
    Elevator *m_elevator;
    Vision *m_vision;

    frc2::CommandXboxController driverCtrlr{0};
    frc2::CommandXboxController operatorCtrlr{1};
    ControllerAxis elevator_nudge_axis;
    ControllerAxis elbow_nudge_axis;
    ControllerAxis climber_nudge_axis;
 
    frc2::Trigger nudge_hold_button;

    frc::SendableChooser<int> m_chooser;

  std::vector<frc2::CommandPtr> AutoCommands;

   struct AutoNameMap {
    std::string Description;
    std::string AutoName;
  };
};
