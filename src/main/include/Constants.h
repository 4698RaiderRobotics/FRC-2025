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

    constexpr int kNumberOfCameras = 3;
    static const CameraLayoutInfo cameraInfo[kNumberOfCameras] = {
        // Front Right Camera
        "CameraA_1MP",
        {frc::Translation3d{-1.9015_in, -10.8912_in, 30.2843_in}, frc::Rotation3d{-8.2028_deg, -23.9351_deg, 36.3362_deg}},
        // Back Left Camera
        "CameraC_2MP",
        {frc::Translation3d(-6.7061_in, 9.7004_in, 24.6382_in), frc::Rotation3d(-8.760_deg, -13.443_deg, -149.618_deg)},
        // Back Right Camera
        "CameraB_2MP",
        {frc::Translation3d{-3.3483_in, -10.3919_in, 26.2791_in}, frc::Rotation3d{9.487_deg, -14.549_deg, 148.395_deg}}
    };

}

    // *****************     ARM SUBSYSTEM      **********************
namespace arm {

    // Maximum angle for the elbow position
    constexpr units::degree_t kElbowMaxAngle = 150_deg;
    // Minimum angle for the elbow position
    constexpr units::degree_t kElbowMinAngle = -38_deg;

    // Elevator Rest Position
    constexpr units::degree_t kElbowRestAngle = 70_deg;

    // Elevator Raise / Lower Position
    constexpr units::degree_t kElbowForwardRaiseAngle = 60_deg;
    constexpr units::degree_t kElbowBackwardRaiseAngle = 130_deg;

    // Elbow Positions for Each Reef Height
     constexpr units::degree_t kElbowCoralL1 = 23_deg;
     constexpr units::degree_t kElbowCoralL2 = 48_deg;
     constexpr units::degree_t kElbowCoralL3 = 48_deg;
     constexpr units::degree_t kElbowCoralL4 = 20_deg;

    // Elbow Positions for Coral Station and Ground Pickup
     constexpr units::degree_t kElbowCoralStation = 145_deg;
     constexpr units::degree_t kElbowGroundPickup = -35_deg;

    // Wrist setpoint Positions 
     constexpr units::degree_t kWristHorizontal = -8_deg;
     constexpr units::degree_t kWristVertical = 98_deg;

}

    // *****************     INTAKE SUBSYSTEM      **********************
namespace intake {

    // Intaking and Ejecting Motor Speed
    constexpr double kIntakeInSpeed = 0.25;
    constexpr double kIntakeOutSpeed = 0.1;

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
    constexpr units::inch_t kHeightCoralL2 = 8.25_in;
    constexpr units::inch_t kHeightCoralL3 = 25.25_in;
    constexpr units::inch_t kHeightCoralL4 = 56_in;

    // Heights for the elevator for Coral Station and Ground Pickup
     constexpr units::inch_t kHeightCoralStation = 15.5_in;
     constexpr units::inch_t kHeightGroundPickup = 0_in;
}

    // *****************     CLIMBER SUBSYSTEM      **********************
namespace climber {

    // Maximum height for the climber position
    constexpr units::inch_t kClimberMaxHeight = 11.5_in;
    // Minimum height for the climber position
    constexpr units::inch_t kClimberMinHeight = 0.0_in;
    // Resting height for the climber 
    constexpr units::inch_t kClimberRestHeight = 1_in;

    // Raising height for the climber 
    constexpr units::inch_t kClimberRaiseHeight = 11_in;
    // Climb height for the climber 
    constexpr units::inch_t kClimberClimbHeight = 4_in;
}

}