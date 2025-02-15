// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "util/LoggedCommand.h"

class Drive;

/**
 * Drive the Robot to a Field coordinate position.
 * 
 * Use trapezoidal profiles for each of the x-axis, y-axis, and rotation.
 */
class DriveToPose
    : public frc2::CommandHelper<LoggedCommand, DriveToPose> {
 public:
  DriveToPose( Drive *drive, frc::Pose2d targetPose);

  void Init() override;

  void Execute() override;

  void Ending(bool interrupted) override;

  bool IsFinished() override;

 private:
  Drive *m_drive;

  frc::Pose2d m_targetPose;
  frc::Pose2d m_profileStartPose;

  units::degree_t m_profileDirection;

  frc::ChassisSpeeds m_speeds;

  frc::TrapezoidProfile<units::meters> m_translationProfile{{12_fps, 10_fps_sq}};
  frc::TrapezoidProfile<units::degrees> m_rotationProfile{{360_deg_per_s, 900_deg_per_s_sq}};

  frc::TrapezoidProfile<units::meters>::State m_translationSetpoint;
  frc::TrapezoidProfile<units::degrees>::State m_rotationSetpoint;

  frc::TrapezoidProfile<units::meters>::State m_translationGoal;
  frc::TrapezoidProfile<units::degrees>::State m_rotationGoal;

  frc::PIDController m_transPID{ 4.0, 0.0, 0.0 };
  frc::PIDController m_rotPID{ 4.0, 0.0, 0.0 };

  units::second_t m_startTime;
};
