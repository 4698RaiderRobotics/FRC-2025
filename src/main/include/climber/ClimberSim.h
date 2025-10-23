#pragma once

#include "ClimberIO.h"

#include "util/AngularSim.h"


class ClimberSim : public ClimberIO {
public:
    ClimberSim();

    void Update( Metrics &m ) override;

    void SetGoal( units::degree_t goal ) override;
    void SetOpenLoop( double percent ) override;
    void ResetAngle( ) override;
    void SetRollers( bool enable ) override;

private:
    AngularSim motorSim;
    bool isHomed{false};
};


