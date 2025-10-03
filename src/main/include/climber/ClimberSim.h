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
    void SetRollers( bool enable ) override;

private:
    LinearSim motorSim;
    bool isHomed{false};
};


