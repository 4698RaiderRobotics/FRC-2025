#pragma once

#include "IntakeIO.h"

#include "util/AngularSim.h"

class IntakeSim : public IntakeIO {
public:
    IntakeSim();
    void Update( Metrics &m ) override;

    void SpinMotors( const SpinSpeed &s ) override;
    void PollingPipeSwitch() override;
private:
    AngularSim upperRoller;
    AngularSim lowerRoller;

    bool centerBeamBlocked = false;
    bool endBeamBlocked = false;
    bool pipeSwitchTripped = false;

    units::second_t start_time;
    bool spinning_in{false};
    bool spinning_out{false};
    bool shifting_up{false};
    bool shifting_down{false};
    bool polling_pipe{false};

    bool isEqual( const SpinSpeed &s1, const SpinSpeed &s2 );
    void StartIntakeEjectState( const SpinSpeed &s );
    void CheckIntakeEjectState();
};