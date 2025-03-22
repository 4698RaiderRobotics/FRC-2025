
#pragma once

#include <functional>

#include <units/length.h>
#include <units/angle.h>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "arm/Arm.h"

#include "util/LoggedCommand.h"

class Elevator;

/**
 * Drive the Robot to a Field pose.
 * 
 * Use trapezoidal profiles for each of the x-axis, y-axis, and rotation.
 */
class MoveMechanism : public frc2::CommandHelper<LoggedCommand, MoveMechanism> {
public:
    MoveMechanism( Arm *arm, Elevator *elevator, units::inch_t height, units::degree_t elbow, ArmIO::WristPosition pos );
    
    void Init() override;

    void Execute() override;

    bool IsFinished() override;

private:
    Arm *m_arm;
    Elevator *m_elevator;

    units::inch_t height_goal;
    units::degree_t elbow_goal;
    ArmIO::WristPosition wrist_goal;

    bool move_arm_forward;
    bool move_arm_backward;
    bool move_elevator_down;
    bool final_move;
};
