#pragma once

#include <units/length.h>

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>
#include <frc/filter/Debouncer.h>

#include "util/Utility.h"

#include "ClimberIO.h"

class Climber : public frc2::SubsystemBase {
public:
    Climber();
    void Periodic() override;

    void SetGoal( units::degree_t goal );
    void Nudge( units::degree_t nudge );
    void SetCageIntake( bool enable_rollers );
    bool AtGoal();
    bool CageLockedIn();
    bool DoingSequence();

    frc2::CommandPtr ChangeHeight( units::degree_t goal );
    frc2::CommandPtr RaiseClimber( );
    frc2::CommandPtr DoClimb( );
    frc2::CommandPtr StopClimber( );

private:
    std::unique_ptr<ClimberIO> io;
    ClimberIO::Metrics metrics;

    util::MotorHomer climbHomer;

    bool rollersEnabled{ false };

    frc::Debouncer cageDeboucer{ 300_ms };

    const units::degree_t AT_GOAL_TOLERANCE = 3_deg;
};
