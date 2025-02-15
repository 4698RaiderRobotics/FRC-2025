#pragma once

#include "IntakeIO.h"

#include "util/AngularSim.h"

class IntakeSim : public IntakeIO {
public:
    IntakeSim();
    void Update( Metrics &m ) override;

    void SpinMotors( double upperSpeed, double lowerSpeed ) override;
private:
    AngularSim upperRoller;
    AngularSim lowerRoller;

    bool centerBeamBlocked = false;
    bool endBeamBlocked = false;
    bool pipeSwitchTripped = false;
};