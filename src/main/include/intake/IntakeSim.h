#pragma once

#include "IntakeIO.h"

#include "util/MotorSim.h"

class IntakeSim : public IntakeIO {
public:
    IntakeSim();
    void Update( Metrics &m ) override;

    void SpinMotors( double upperSpeed, double lowerSpeed ) override;
private:
    MotorSim upperRoller;
    MotorSim lowerRoller;

    bool centerBeamBlocked = false;
    bool endBeamBlocked = false;
    bool pipeSwitchTripped = false;
};