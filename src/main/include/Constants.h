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
        {frc::Translation3d{10.8079_in, 7.48097_in, 7.89409_in}, frc::Rotation3d{3.10403_deg, -14.84949_deg, -46.8458_deg}},
        // Front Right Camera
        "Front_Right",
        {frc::Translation3d(10.33464_in, -6.32108_in, 8.60338_in), frc::Rotation3d(0.274182_deg, -14.47084_deg, 44.39811_deg)},
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
    constexpr units::degree_t kElbowHomingRestAngle = 115_deg;

    // Elevator Raise / Lower Position
    constexpr units::degree_t kElbowForwardRaiseAngle = 60_deg;
    constexpr units::degree_t kElbowBackwardRaiseAngle = 125_deg;

    // Elbow Positions for Each Reef Height
     constexpr units::degree_t kElbowCoralL1 = 28_deg;
     constexpr units::degree_t kElbowCoralL2 = 42_deg;
     constexpr units::degree_t kElbowCoralL3 = 42_deg;
     constexpr units::degree_t kElbowCoralL4 = 20_deg;

    // Elbow Positions for Coral Station and Ground Pickup
     constexpr units::degree_t kElbowCoralStation = 145_deg;
     constexpr units::degree_t kElbowGroundPickup = -35_deg;

    // Elbow Position for Removing Algae
     constexpr units::degree_t kElbowRemoveAlgaeLow = 60_deg;
     constexpr units::degree_t kElbowRemoveAlgaeHigh = 53_deg;
     constexpr units::degree_t kElbowAlgaeHoldingPos = 51_deg;

    // Wrist setpoint Positions 
     constexpr units::degree_t kWristHorizontal = -5_deg;
     constexpr units::degree_t kWristVertical = 110_deg;

}

    // *****************     INTAKE SUBSYSTEM      **********************
namespace intake {

    // Intaking and Ejecting Motor Speed
    constexpr double kIntakeInSpeed = 0.30;
    constexpr double kIntakeOutSpeed = 0.15;

    // Shift speed of the motors
    constexpr double kIntakeShiftFastSpeed = 0.5;
    constexpr double kIntakeShiftSlowSpeed = 0.35;
}


    // *****************     ELEVATOR SUBSYSTEM      **********************
namespace elevator {

    // Maximum height for the elevator position
    constexpr units::inch_t kElevatorMaxHeight = 57_in;
    // Minimum height for the elevator position
    constexpr units::inch_t kElevatorMinHeight = 0_in;

    // Heights for the elevator to place on Reef
    constexpr units::inch_t kHeightCoralL1 = 0_in;
    constexpr units::inch_t kHeightCoralL2 = 9_in;
    constexpr units::inch_t kHeightCoralL3 = 25.25_in;
    constexpr units::inch_t kHeightCoralL4 = 56_in;

    // Heights for the elevator for Coral Station and Ground Pickup
     constexpr units::inch_t kHeightCoralStation = 15.5_in;
     constexpr units::inch_t kHeightGroundPickup = 0_in;

    // Heights for removing Algae
    constexpr units::inch_t kHeightRemoveAlgaeLow = 0_in;
    constexpr units::inch_t kHeightRemoveAlgaeHigh = 18_in;

    constexpr units::inch_t kHeightDislogAlgae = 39_in;
}

    // *****************     CLIMBER SUBSYSTEM      **********************
namespace climber {

    // Maximum height for the climber position
    constexpr units::inch_t kClimberMaxHeight = 5_in;
    // Minimum height for the climber position
    constexpr units::inch_t kClimberMinHeight = -6.5_in;
    // Resting height for the climber 
    constexpr units::inch_t kClimberRestHeight = 0_in;

    // Raising height for the climber 
    constexpr units::inch_t kClimberRaiseHeight = 4.75_in;
    // Climb height for the climber 
    constexpr units::inch_t kClimberClimbHeight = -5.75_in;
}

    // *****************     REEF PLACEMENT      **********************
namespace reef {

        // NOTE:
        // If pose_shift_out or pose_shift_lateral is changed then the PathPlanner
        // JSON data for the pose locations must be regenerated for autos to work correctly.
        // See ReefPlacingPoses::OutputPathPlannerJSON()
    constexpr units::inch_t pose_shift_out = 23_in;
    constexpr units::inch_t pose_shift_lateral = 6.5_in;
    constexpr units::inch_t algae_remove_pose_shift_out = 21_in;

    // Position away from April Tag to place/remove.
    constexpr units::inch_t place_L1 = 21_in;
    constexpr units::inch_t place_L2 = 18_in;
    constexpr units::inch_t place_L3 = 14.5_in;
    constexpr units::inch_t place_L4 = 10_in;
    constexpr units::inch_t remove_algae = 16_in;

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