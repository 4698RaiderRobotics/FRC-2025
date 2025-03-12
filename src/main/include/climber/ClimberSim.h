#pragma once

#include "ClimberIO.h"

#include "util/LinearSim.h"


class ClimberSim : public ClimberIO {
public:
    ClimberSim();

    void Update( Metrics &m ) override;

    void SetGoal( units::inch_t goal ) override;
    void SetOpenLoop( double percent ) override;
    void ResetHeight( ) override;

private:
    LinearSim motorSim;
};


