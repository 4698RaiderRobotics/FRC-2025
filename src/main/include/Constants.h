#pragma once

#include <numbers>
#include <units/length.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <frc/geometry/Transform3d.h>

namespace physical {

namespace vision {

    struct CameraLayoutInfo {
        const char *name;
        const frc::Transform3d robotToCamera;
    };

    constexpr int kNumberOfCameras = 2;
    static const CameraLayoutInfo cameraInfo[kNumberOfCameras] = {
        // Front Left Camera
        "Front_Left",
        // {frc::Translation3d{10.8079_in, 7.48097_in, 7.89409_in}, frc::Rotation3d{3.10403_deg, -14.84949_deg, -46.8458_deg}},
        { frc::Translation3d{12.6850181_in, 11.9340367_in, 8.17935338_in}, frc::Rotation3d{-0.483946556_deg, -23.4951297_deg, -0.362283408_deg} },
        // Front Right Camera
        "Front_Right",
        // {frc::Translation3d(10.33464_in, -6.32108_in, 8.60338_in), frc::Rotation3d(0.274182_deg, -14.47084_deg, 44.39811_deg)},
        { frc::Translation3d(12.7645852_in, -11.689974_in, 8.11366514_in), frc::Rotation3d(1.21045498_deg, -23.4920369_deg, -0.168526218_deg) },
        // Back Right Camera
        // "CameraB_2MP",
        // {frc::Translation3d{-3.3483_in, -10.3919_in, 26.2791_in}, frc::Rotation3d{9.487_deg, -14.549_deg, 148.395_deg}}
    };

}

    // *****************     ARM SUBSYSTEM      **********************
namespace arm {

    // Maximum angle for the elbow position
    constexpr units::degree_t kElbowMaxAngle = 150_deg;
    // Minimum angle for the elbow position
    constexpr units::degree_t kElbowMinAngle = -38_deg;

    // Elevator Rest Position
    constexpr units::degree_t kElbowRestAngle = 125_deg;
    constexpr units::degree_t kElbowHomingRestAngle = 120_deg;
    constexpr units::degree_t kCoralHoldPos = 78_deg;

    // Elevator Raise / Lower Position
    constexpr units::degree_t kElbowForwardRaiseAngle = 50_deg;
    constexpr units::degree_t kElbowBackwardRaiseAngle = 125_deg;

    // Elbow Positions for Each Reef Height
     constexpr units::degree_t kElbowCoralL1 = 28_deg;
     constexpr units::degree_t kElbowCoralL2 = 42_deg;
     constexpr units::degree_t kElbowCoralL3 = 42_deg;
     constexpr units::degree_t kElbowCoralL4 = 23_deg;

    // Elbow Positions for Coral Station and Ground Pickup
     constexpr units::degree_t kElbowCoralStation = 138_deg;
     constexpr units::degree_t kElbowGroundPickup = -30_deg;

    // Elbow Position for Removing Algae
     constexpr units::degree_t kElbowRemoveAlgaeLow = 60_deg;
     constexpr units::degree_t kElbowRemoveAlgaeHigh = 40_deg;
     constexpr units::degree_t kElbowAlgaeHoldingPos = 30_deg;
     constexpr units::degree_t kElbowAlgaeRemoveEnd = 85_deg;
     constexpr units::degree_t kElbowHighAlgaeEnd = 80_deg;

    // Wrist setpoint Positions 
     constexpr units::degree_t kWristHorizontal = -5_deg;
     constexpr units::degree_t kWristVertical = 110_deg;

}

    // *****************     INTAKE SUBSYSTEM      **********************
namespace intake {

    // Intaking and Ejecting Motor Speed
    // Multiply for minions 
    constexpr double minion_mult = 2.0;
    constexpr double kIntakeInSpeed = 0.30 * minion_mult;
    constexpr double kIntakeOutSpeed = 0.15 * minion_mult;

    // Shift speed of the motors
    constexpr double kIntakeShiftFastSpeed = 0.5 * minion_mult;
    constexpr double kIntakeShiftSlowSpeed = 0.35 * minion_mult;
}


    // *****************     ELEVATOR SUBSYSTEM      **********************
namespace elevator {

    // Maximum height for the elevator position
    constexpr units::inch_t kElevatorMaxHeight = 57.9_in;
    // Minimum height for the elevator position
    constexpr units::inch_t kElevatorMinHeight = 0_in;

    // Heights for the elevator to place on Reef
    constexpr units::inch_t kHeightCoralL1 = 1.5_in;
    constexpr units::inch_t kHeightCoralL2 = 10.5_in;
    constexpr units::inch_t kHeightCoralL3 = 25.75_in;
    constexpr units::inch_t kHeightCoralL4 = 56.5_in;

    // Heights for the elevator for Coral Station and Ground Pickup
     constexpr units::inch_t kHeightCoralStation = 7.5_in;
     constexpr units::inch_t kHeightGroundPickup = 0_in;

    // Heights for removing Algae
    constexpr units::inch_t kHeightRemoveAlgaeLow = 0_in;
    constexpr units::inch_t kHeightRemoveAlgaeHigh = 15_in;
    constexpr units::inch_t kHeightRemoveHighAlgaeEnd = 17_in;

    constexpr units::inch_t kHeightDislogAlgae = 39_in;
}

    // *****************     CLIMBER SUBSYSTEM      **********************
namespace climber {

    // Maximum height for the climber position
    constexpr units::degree_t kClimberMaxAngle = 100_deg;
    // Minimum height for the climber position
    constexpr units::degree_t kClimberMinAngle = -20_deg;
    // Resting height for the climber 
    constexpr units::degree_t kClimberRestAngle = -5_deg;

    // Raising height for the climber 
    constexpr units::degree_t kClimberRaiseAngle = 95_deg;
    // Climb height for the climber 
    constexpr units::degree_t kClimberClimbAngle = -5_deg;
}

    // *****************     REEF PLACEMENT      **********************
namespace reef {

        // NOTE:
        // If pose_shift_out or pose_shift_lateral is changed then the PathPlanner
        // JSON data for the pose locations must be regenerated for autos to work correctly.
        // See ReefPlacingPoses::OutputPathPlannerJSON()
    constexpr units::inch_t pose_shift_out = 23_in;
    constexpr units::inch_t pose_shift_lateral = 6.5_in;
    constexpr units::inch_t algae_remove_pose_shift_out = 31_in;

    // Position away from April Tag to place/remove.
    constexpr units::inch_t place_L1 = 25_in;
    constexpr units::inch_t place_L2 = 18_in;
    constexpr units::inch_t place_L3 = 14.5_in;
    constexpr units::inch_t place_L4 = 14_in;
    constexpr units::inch_t remove_algae = 13_in;

    // const units::inch_t place_L1_shift_in = 0_in;
    // const units::inch_t place_L2_shift_in = 6_in;
    // const units::inch_t place_L3_shift_in = 8.5_in;
    // const units::inch_t place_L4_shift_in = 13_in;
    // const units::inch_t algae_shift_in = 5_in;

    constexpr units::inch_t place_L1_shift_in = pose_shift_out - place_L1;
    constexpr units::inch_t place_L2_shift_in = pose_shift_out - place_L2;
    constexpr units::inch_t place_L3_shift_in = pose_shift_out - place_L3;
    constexpr units::inch_t place_L4_shift_in = pose_shift_out - place_L4;
    constexpr units::inch_t algae_shift_in = algae_remove_pose_shift_out - remove_algae;
}

}