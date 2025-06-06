#pragma once

#include <units/length.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>

#include "util/Utility.h"

#include "ClimberIO.h"

class Climber : public frc2::SubsystemBase {
public:
    Climber();
    void Periodic() override;

    void SetGoal( units::inch_t goal );
    void Nudge( units::inch_t nudge );
    bool AtGoal();

    // frc2::CommandPtr ChangeHeight( units::inch_t goal );
    frc2::CommandPtr RaiseClimber( );
    frc2::CommandPtr DoClimb( );

    frc2::Trigger isHoming();

private:
    std::unique_ptr<ClimberIO> io;
    ClimberIO::Metrics metrics;

    util::MotorHomer climbHomer;

    const units::inch_t AT_GOAL_TOLERANCE = 1_in;
};
