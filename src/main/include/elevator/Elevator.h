#pragma once

#include <units/length.h>

#include <frc2/command/SubsystemBase.h>

#include "ElevatorIO.h"

#include "util/Utility.h"

class Elevator : public frc2::SubsystemBase {
public:
    Elevator();
    void Periodic() override;

    void SetGoal( units::inch_t goal );
    void Nudge( units::inch_t nudge );
    bool AtGoal();
    units::inch_t GetHeight() {return metrics.height;}

    frc2::CommandPtr ChangeHeight( units::inch_t goal );

private:
    std::unique_ptr<ElevatorIO> io;
    ElevatorIO::Metrics metrics;

    util::MotorHomer homer;

    const units::inch_t AT_GOAL_TOLERANCE = 1_in;
};
