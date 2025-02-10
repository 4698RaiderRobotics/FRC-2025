#pragma once

#include "ElevatorIO.h"

#include "util/MotorSim.h"


class ElevatorSim : public ElevatorIO {
public:
    ElevatorSim();

    void Update( Metrics &m ) override;

    void SetGoal( units::inch_t goal ) override;
private:
    MotorSim motorSim;
};


