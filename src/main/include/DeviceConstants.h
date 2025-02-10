#pragma once

#include <numbers>

#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>

#include "util/Tuning.h"

namespace device {

    // *****************     ARM SUBSYSTEM      **********************
namespace arm {

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
}


    // *****************     ELEVATOR SUBSYSTEM      **********************
namespace elevator {

    // Maximum velocity for the elevator height TrapezoidProfile
    // constexpr units::meters_per_second_t kElevatorMaxSpeed = 2_mps;
    // Maximum acceleration for the elevator height TrapezoidProfile
    // constexpr units::meters_per_second_squared_t kElevatorMaxAcceleration = 4_mps_sq;

    constexpr units::inch_t kGearDiameter = 1.5_in;
    constexpr double kGearRatio = 15;

        // Compound unit for the inches per revolution constant.
    using inches_per_rev = units::compound_unit<units::inches, units::inverse<units::turns>>;
    using inches_per_rev_t = units::unit_t<inches_per_rev>;

        // The number of inches traveled per rotation of the motor
        // wheel circumference / gear ratio
    constexpr inches_per_rev_t kDistancePerMotorRev = std::numbers::pi * kGearDiameter / ( kGearRatio *  1_tr );

    constexpr MotionConfig<units::inches> kMotionConfig = {
        { 5.5, 0.0, 0.5, 0.1, 0.45, 9.0, 0.0 },
        { 2_fps, 4_fps_sq, 0_fps_cu }
    };
}


    // *****************     INTAKE SUBSYSTEM      **********************
namespace intake {

    constexpr double kGearRatio = 4.5;
}


    // *****************     CLIMBER SUBSYSTEM      **********************
namespace climber {

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
}

namespace deviceIDs {
    constexpr int kRightShooterID = 14;
    constexpr int kLeftShooterID = 15;
    constexpr int kShooterAngleID = 16;

    constexpr int kArmMotorID = 21;
    constexpr int kWristMotorID = 22;

    constexpr int kElevatorID = 18;

    constexpr int kClimberID = 19;

    constexpr int kIntakeUpperMotorID = 28;
    constexpr int kIntakeLowerMotorID = 29;
    constexpr int kIntakeCenterSensorPort = 0;
    constexpr int kIntakeEndSensorPort = 1;
    constexpr int kIntakePipeSwitchPort = 2;

    constexpr int kShooterEncoderID = 17;
    constexpr int kArmEncoderID = 23;
    constexpr int kWristEncoderID = 24;
}

