#pragma once

#include "ElevatorIO.h"

class ElevatorTalon : public ElevatorIO {
public:
    ElevatorTalon( int canId );

    void UpdateMetrics( Metrics &m );

    void SetGoal( units::inch_t goal );
private:
    // motors etc.
};
