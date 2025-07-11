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
    constexpr double kElbowGearRatio = 36;

        // Wrist Gear Ratio
    constexpr double kWristGearRatio = 45;

        // PIDSGVA and Motion Profile settings
    constexpr MotionConfig<units::turns> kElbowMotionConfig = {
        // { 20.0, 0.0, 5.0, 0.05, 0.47, 4.5, 0.0 },
        // { 2_tps, 5_tr_per_s_sq, 20_tr_per_s_cu }
        { 20.0, 0.0, 5.0, 0.05, 0.47, 4.5, 0.0 },
        { 1_tps, 2_tr_per_s_sq, 10_tr_per_s_cu }

    };
    constexpr MotionConfig<units::turns> kWristMotionConfig = {
        // { 0.01, 0.0, 0.0, 0.0, 0.0, 0.0094, 0.0 },
        // { 0.75_tps, 2_tr_per_s_sq, 0_tr_per_s_cu }
        { 0.01, 0.0, 0.0, 0.0, 0.0, 0.0094, 0.0 },
        { 0.3_tps, 1_tr_per_s_sq, 0_tr_per_s_cu }
    };
}


    // *****************     ELEVATOR SUBSYSTEM      **********************
namespace elevator {

        // Pulley diameter HTD 24T
    constexpr units::inch_t kGearDiameter = 1.504_in;

        // Gear Ratio
    constexpr double kGearRatio = 5;

        // The number of inches traveled per rotation of the motor
        // wheel circumference / gear ratio
    constexpr inches_per_rev_t kDistancePerMotorRev = std::numbers::pi * kGearDiameter * 2.0 / ( kGearRatio *  1_tr );

        // PIDSGVA and Motion Profile settings
    constexpr MotionConfig<units::inches> kMotionConfig = {
        // { 7.5, 0.0, 0.0, 0.175, 0.375, 0.125, 0.0 },
        // { 4.4_mps, 8_mps_sq, 60_mps_cu }
        { 7.5, 0.0, 0.0, 0.175, 0.375, 0.125, 0.0 },
        { 1.5_mps, 3_mps_sq, 10_mps_cu }
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
    constexpr double kGearRatio = 45 * 38.0 / 34.0;

        // The number of inches traveled per rotation of the motor
        // wheel circumference / gear ratio
    constexpr inches_per_rev_t kDistancePerMotorRev = std::numbers::pi * kSpoolDiameter / ( kGearRatio *  1_tr );

        // PIDSGVA and Motion Profile settings
    constexpr MotionConfig<units::meters> kMotionConfig = {
        { 8.0, 0.0, 1.0, 0.0, 0.0, 40.0, 0.0 },
        { 0.22_mps, 3_mps_sq, 0_mps_cu }
    };
}
}

namespace deviceIDs {
    constexpr int kElbowMotorID = 21;
    constexpr int kWristMotorID = 25;
    constexpr int kElbowEncoderID = 26;

    constexpr int kElevatorID = 20;
    constexpr int kElevatorID2 = 27;

    constexpr int kClimberID = 24;

    constexpr int kIntakeUpperMotorID = 23;
    constexpr int kIntakeLowerMotorID = 22;

    constexpr int kIntakeCenterSensorPort = 0;
    constexpr int kIntakeEndSensorPort = 1;
    constexpr int kIntakePipeSwitchPort = 99;

    constexpr int kClimberHomeSwitch = 2;

}

