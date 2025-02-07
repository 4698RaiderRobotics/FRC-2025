#pragma once

#include "IntakeIO.h"

#include <frc/simulation/DCMotorSim.h>

class IntakeSim : public IntakeIO {
public:
    void UpdateMetrics( Metrics &m ) override;

    void SpinMotors( double upperSpeed, double lowerSpeed ) override;
    bool isCenterBroken( ) override { return centerBeamState; }
    bool isEndBroken( ) override { return endBeamState; }
private:
    frc::sim::DCMotorSim upperRoller;
    frc::sim::DCMotorSim lowerRoller;

    bool centerBeamState = false;
    bool endBeamState = false;
};