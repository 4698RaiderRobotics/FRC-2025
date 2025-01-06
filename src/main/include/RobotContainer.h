// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>

#include <frc2/command/button/CommandPS5Controller.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "ControllerAxis.h"
#include "subsystems/SwerveDriveSubsystem.h"

//#include "commands/Composite.h"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();

  
  frc::SendableChooser<int> m_chooser;

  std::vector<frc2::CommandPtr> AutoCommands;


  SwerveDriveSubsystem m_swerveDrive;

  frc2::CommandPS5Controller m_driverController{0};
  frc2::CommandXboxController m_operatorController{1};

  ControllerAxis vx_axis{m_driverController, frc::PS5Controller::Axis::kLeftY, true};
  ControllerAxis vy_axis{m_driverController, frc::PS5Controller::Axis::kLeftX, true};
  ControllerAxis omega_axis{m_driverController, frc::PS5Controller::Axis::kRightX, true};
  ControllerAxis arm_angle_axis{ m_operatorController, frc::XboxController::Axis::kLeftY, true };
  ControllerAxis wrist_angle_axis{ m_operatorController, frc::XboxController::Axis::kRightY, true };
  ControllerAxis elevator_axis{ m_operatorController, frc::XboxController::Axis::kLeftX, true };
};
