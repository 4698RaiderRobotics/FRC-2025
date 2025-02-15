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

        // Camera "CameraA_1MP"
    const frc::Transform3d kFrontRightRobotToCam = 
            frc::Transform3d(frc::Translation3d{-1.9015_in, -10.8912_in, 30.2843_in}, frc::Rotation3d{-8.2028_deg, -23.9351_deg, 36.3362_deg});

         // Camera "CameraB_2MP"
   const frc::Transform3d kBackRightRobotToCam = 
            frc::Transform3d(frc::Translation3d{-3.3483_in, -10.3919_in, 26.2791_in}, frc::Rotation3d{9.487_deg, -14.549_deg, 148.395_deg});

         // Camera "CameraC_2MP"
    const frc::Transform3d kBackLeftRobotToCam = 
            frc::Transform3d(frc::Translation3d(-6.7061_in, 9.7004_in, 24.6382_in), frc::Rotation3d(-8.760_deg, -13.443_deg, -149.618_deg));
}

    // *****************     ARM SUBSYSTEM      **********************
namespace arm {

    // Maximum angle for the elbow position
    constexpr units::degree_t kElbowMaxAngle = 120_deg;
    // Minimum angle for the elbow position
    constexpr units::degree_t kElbowMinAngle = -15_deg;

    // Elevator Raise / Lower Position
    constexpr units::degree_t kElbowRaiseAngle = 60_deg;

    // Elbow Positions for Each Reef Height
     constexpr units::degree_t kElbowCoralL1 = 0_deg;
     constexpr units::degree_t kElbowCoralL2 = 25_deg;
     constexpr units::degree_t kElbowCoralL3 = 25_deg;
     constexpr units::degree_t kElbowCoralL4 = 5_deg;

}

    // *****************     INTAKE SUBSYSTEM      **********************
namespace intake {

    // Intaking and Ejecting Motor Speed
    constexpr double kIntakeInOutSpeed = 0.75;
    // Shift speed of the slower motor
    constexpr double kIntakeShiftSpeed = 0.5;
}


    // *****************     ELEVATOR SUBSYSTEM      **********************
namespace elevator {

    // Maximum height for the elevator position
    constexpr units::inch_t kElevatorMaxHeight = 65_in;
    // Minimum height for the elevator position
    constexpr units::inch_t kElevatorMinHeight = 0_in;

    // Height for the elevator to place on Reef
    constexpr units::inch_t kCoralL1Height = 15_in;
    constexpr units::inch_t kCoralL2Height = 25_in;
    constexpr units::inch_t kCoralL3Height = 40_in;
    constexpr units::inch_t kCoralL4Height = 65_in;
}

    // *****************     CLIMBER SUBSYSTEM      **********************
namespace climber {

    // Maximum height for the climber position
    constexpr units::inch_t kClimberMaxHeight = 19.5_in;
    // Minimum height for the climber position
    constexpr units::inch_t kClimberMinHeight = 0.0_in;
    // Resting height for the climber 
    constexpr units::inch_t kClimberRestHeight = 1_in;

}

}