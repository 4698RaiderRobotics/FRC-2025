#pragma once

#include "ClimberIO.h"

#include "util/LinearSim.h"


class ClimberSim : public ClimberIO {
public:
    ClimberSim();

    void Update( Metrics &m ) override;

    void SetGoal( units::inch_t goal ) override;
private:
    LinearSim motorSim;
};


