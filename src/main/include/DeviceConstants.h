#pragma once

#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

namespace pidf {
    // *****************     SHOOTER SUBSYSTEM      **********************

    // Motion Profile for the shooter angle TrapezoidProfile
    constexpr units::degrees_per_second_t kShooterMaxSpeed = 360_deg_per_s;
    constexpr units::degrees_per_second_squared_t kShooterMaxAcceleration = 360_deg_per_s_sq;

    constexpr double kShooterP = 0.02;
    constexpr double kShooterI = 0.0;
    constexpr double kShooterD = 0.0;

    constexpr double kShooterS = 0.0;
    constexpr double kShooterG = 0.1;
    constexpr double kShooterV = 8.0;
    constexpr double kShooterA = 0.0;


    // *****************     ARM SUBSYSTEM      **********************

    // Maximum velocity for the arm angle TrapezoidProfile
    constexpr units::turns_per_second_t kArmMaxSpeed = 2_tps;
    // Maximum acceleration for the arm angle TrapezoidProfile
    constexpr units::turns_per_second_squared_t kArmMaxAcceleration = 4_tr_per_s_sq;
    // Maximum velocity for the wrist MotionMagic profile
    // Units in rotations per second
    constexpr double kWristMaxSpeed = 2;
    // Maximum acceleration for the wrist MotionMagic profile
    // Units in rotations per second squared
    constexpr double kWristMaxAcceleration = 4;

    constexpr double kArmP = 0.01;
    constexpr double kArmI = 0.0;
    constexpr double kArmD = 0.0;

    constexpr double kArmS = 0.0;
    constexpr double kArmG = 0.3;
    constexpr double kArmWristG = 0.01;
    constexpr double kArmV = 0.46;
    constexpr double kArmA = 0.0;

    constexpr double kWristP = 1.5;
    constexpr double kWristI = 0.0;
    constexpr double kWristD = 0.1;

    constexpr double kWristS = 0.0;
    constexpr double kWristG = 0.022;
    constexpr double kWristV = 0.2;
    constexpr double kWristA = 0.0;


    // *****************     ELEVATOR SUBSYSTEM      **********************

    // Maximum velocity for the elevator height TrapezoidProfile
    constexpr units::meters_per_second_t kElevatorMaxSpeed = 2_mps;
    // Maximum acceleration for the elevator height TrapezoidProfile
    constexpr units::meters_per_second_squared_t kElevatorMaxAcceleration = 4_mps_sq;

    constexpr double kElevatorP = 5.5;
    constexpr double kElevatorI = 0.0;
    constexpr double kElevatorD = 0.5;

    constexpr double kElevatorS = 0.1;
    constexpr double kElevatorG = 0.45;
    constexpr double kElevatorV = 9.0;
    constexpr double kElevatorA = 0.0;

    constexpr double kSpeedP = 0.001;
    constexpr double kSpeedI = 0.0;
    constexpr double kSpeedD = 0.0;
    constexpr double kSpeedFF = 0.00018;


    // *****************     CLIMBER SUBSYSTEM      **********************

    // Maximum velocity for the climber motion TrapezoidProfile
    constexpr units::meters_per_second_t kClimberMaxSpeed = 0.25_mps;
    // Maximum acceleration for the climber motion TrapezoidProfile
    constexpr units::meters_per_second_squared_t kClimberMaxAcceleration = 2_mps_sq;

    constexpr double kClimberP = 1.5;
    constexpr double kClimberI = 0.0;
    constexpr double kClimberD = 0.0;

    constexpr double kClimberS = 0.0;
    constexpr double kClimberV = 34.0;
    constexpr double kClimberA = 0.0;
}

namespace deviceIDs {
    constexpr int kRightShooterID = 14;
    constexpr int kLeftShooterID = 15;
    constexpr int kShooterAngleID = 16;

    constexpr int kArmMotorID = 21;
    constexpr int kWristMotorID = 22;

    constexpr int kElevatorID = 18;

    constexpr int kClimberID = 19;

    constexpr int kIntakeID = 20;

    constexpr int kShooterEncoderID = 17;
    constexpr int kArmEncoderID = 23;
    constexpr int kWristEncoderID = 24;
}

