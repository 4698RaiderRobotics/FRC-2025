#pragma once

#include <frc2/command/button/CommandXBoxController.h>

namespace ctrl {
    constexpr int drive_X_axis = frc::XboxController::Axis::kLeftY;
    constexpr int drive_Y_axis = frc::XboxController::Axis::kLeftX;
    constexpr int drive_theta_axis = frc::XboxController::Axis::kRightX;

    constexpr int intake_from_coral_station = frc::XboxController::Button::kB;
    constexpr int intake_from_ground = frc::XboxController::Button::kA;

    constexpr int pick_L1_level = frc::XboxController::Button::kA;
    constexpr int pick_L2_level = frc::XboxController::Button::kB;
    constexpr int pick_L3_level = frc::XboxController::Button::kY;
    constexpr int pick_L4_level = frc::XboxController::Button::kRightBumper;
    constexpr int place_on_reef = frc::XboxController::Axis::kRightTrigger;

    constexpr int nudge_modifier = frc::XboxController::Button::kLeftBumper;
    constexpr int nudge_elevator_axis = frc::XboxController::Axis::kLeftY;
    constexpr int nudge_elbow_axis = frc::XboxController::Axis::kRightX;
    constexpr int nudge_wrist_axis = frc::XboxController::Axis::kRightY;
    constexpr int nudge_intake_spin_in = frc::XboxController::Button::kB;
    constexpr int nudge_intake_spin_out = frc::XboxController::Button::kX;
    constexpr int nudge_intake_shift_up = frc::XboxController::Button::kY;
    constexpr int nudge_intake_shift_down = frc::XboxController::Button::kA;
    constexpr int nudge_climber_axis = frc::XboxController::Axis::kLeftX;
}