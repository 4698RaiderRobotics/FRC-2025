#pragma once

#include "ArmIO.h"

#include "util/AngularSim.h"


class ArmSim : public ArmIO {
public:
    ArmSim();

    void Update( Metrics &m ) override;

    void SetElbowGoal( units::degree_t goal ) override;
    void SetWristPosition( WristPosition pos ) override;
    void ResetWristAngle( units::degree_t position ) override;
    void SetWristOpenLoop( double percentOutput ) override;
private:
    AngularSim elbowSim;
    AngularSim wristSim;
};


