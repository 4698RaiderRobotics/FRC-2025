#pragma once

#include <units/angle.h>

#include <frc2/command/SubsystemBase.h>

#include "ArmIO.h"

class Arm : public frc2::SubsystemBase {
public:
    Arm();
    void Periodic() override;

    void SetGoal( units::degree_t goal );
    void Nudge( units::degree_t nudge );
    bool AtGoal();

    frc2::CommandPtr ChangeAngle( units::degree_t goal );

private:
    std::unique_ptr<ArmIO> io;
    ArmIO::Metrics metrics;

    const units::degree_t AT_GOAL_TOLERANCE = 4_deg;
};
