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

    // Compound unit for the inches per revolution constant.
using inches_per_rev = units::compound_unit<units::inches, units::inverse<units::turns>>;
using inches_per_rev_t = units::unit_t<inches_per_rev>;


namespace device {

    // *****************     ARM SUBSYSTEM      **********************
namespace arm {

        // Elbow Gear Ratio
    constexpr double kElbowGearRatio = 15;

        // Wrist Gear Ratio
    constexpr double kWristGearRatio = 45;

        // PIDSGVA and Motion Profile settings
    constexpr MotionConfig<units::turns> kElbowMotionConfig = {
        { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 },
        { 2_tps, 4_tr_per_s_sq, 0_tr_per_s_cu }
    };
    constexpr MotionConfig<units::turns> kWristMotionConfig = {
        { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 },
        { 2_tps, 4_tr_per_s_sq, 0_tr_per_s_cu }
    };
}


    // *****************     ELEVATOR SUBSYSTEM      **********************
namespace elevator {

        // Pulley diameter
    constexpr units::inch_t kGearDiameter = 1.5_in;

        // Gear Ratio
    constexpr double kGearRatio = 15;

        // The number of inches traveled per rotation of the motor
        // wheel circumference / gear ratio
    constexpr inches_per_rev_t kDistancePerMotorRev = std::numbers::pi * kGearDiameter / ( kGearRatio *  1_tr );

        // PIDSGVA and Motion Profile settings
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

       // Spool diameter
    constexpr units::inch_t kSpoolDiameter = 0.75_in;

        // Gear Ratio
    constexpr double kGearRatio = 15;

        // The number of inches traveled per rotation of the motor
        // wheel circumference / gear ratio
    constexpr inches_per_rev_t kDistancePerMotorRev = std::numbers::pi * kSpoolDiameter / ( kGearRatio *  1_tr );

        // PIDSGVA and Motion Profile settings
    constexpr MotionConfig<units::inches> kMotionConfig = {
        { 1.5, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 },
        { 0.5_fps, 2_fps_sq, 0_fps_cu }
    };
}
}

namespace deviceIDs {
    constexpr int kElbowMotorID = 21;
    constexpr int kWristMotorID = 22;
    constexpr int kElbowEncoderID = 23;

    constexpr int kElevatorID = 18;

    constexpr int kClimberID = 19;

    constexpr int kIntakeUpperMotorID = 28;
    constexpr int kIntakeLowerMotorID = 29;

    constexpr int kIntakeCenterSensorPort = 0;
    constexpr int kIntakeEndSensorPort = 1;
    constexpr int kIntakePipeSwitchPort = 2;
}

