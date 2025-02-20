#pragma once

#include "ElevatorIO.h"

#include "util/LinearSim.h"


class ElevatorSim : public ElevatorIO {
public:
    ElevatorSim();

    void Update( Metrics &m ) override;

    void ResetPosition( units::inch_t position ) override;
    void SetOpenLoop( double percentOutput ) override;
    void SetGoal( units::inch_t goal ) override;
private:
    LinearSim motorSim;
};


