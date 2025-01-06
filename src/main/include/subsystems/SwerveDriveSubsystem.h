// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <wpi/array.h>
#include <wpi/DataLog.h>

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/controller/HolonomicDriveController.h>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <units/time.h>
using namespace units::literals;

#include "SwerveModule.h"
// #include "SwerveModuleDisplay.h"
//#include "subsystems/VisionSubsystem.h"

class SwerveDriveSubsystem : public frc2::SubsystemBase {
 public:
  
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  SwerveDriveSubsystem();
    
    void ArcadeDrive( double xPercent, double yPercent, double omegaPercent, bool operatorRelative = true );
    void Drive( frc::ChassisSpeeds speeds, bool fieldRelative = true );

    void DriveTrajectory( frc::Trajectory::State trajState, const frc::Rotation2d &robotHeading );

    void Periodic( void );

    frc::ChassisSpeeds GetRobotRelativeSpeeds();

    frc::Pose2d GetPose( void );

    void ResetGyro( units::degree_t angle );

    void ResetDriverOrientation( units::degree_t angle );

    void ResetPose( frc::Pose2d pose );

  private:
    //VisionSubsystem *m_vision;

    SwerveModule m_modules[4];

    frc::Trajectory m_trajectory;
    wpi::array<frc::SwerveModuleState, 4U> m_desiredStates{ wpi::empty_array };
    wpi::array<frc::SwerveModuleState, 4U> m_actualStates{ wpi::empty_array };


    ctre::phoenix6::hardware::Pigeon2 m_gyro;

    frc::Translation2d m_mod_Location[4];

    frc::SwerveDriveKinematics<4> m_kinematics;
    frc::SwerveDrivePoseEstimator<4> m_odometry;

    frc::ChassisSpeeds m_robotRelativeSpeeds;

    // Drive controller for driving a trajectory
    frc::HolonomicDriveController m_controller;

    units::degree_t field_offset;
    units::degree_t driver_offset;
    bool m_have_driver_offset{ false  };

  public:
    frc::Field2d m_field;
    frc::Trajectory exampleTraj;
};
