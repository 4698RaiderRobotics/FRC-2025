#pragma once

#include "ArmIO.h"

#include "util/AngularSim.h"


class ArmSim : public ArmIO {
public:
    ArmSim();

    void Update( Metrics &m ) override;

    void SetElbowGoal( units::degree_t goal ) override;
    void SetWristHorizontal( WristPosition pos ) override;
private:
    AngularSim elbowSim;
    AngularSim wristSim;
};


