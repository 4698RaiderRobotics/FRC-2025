#pragma once

#include "IntakeIO.h"


class Intake550 : public IntakeIO {
public:
    Intake550( int upperCanID, int lowerCanID, int centerPort, int endPort );
    void UpdateMetrics( Metrics &m ) override;
    
    void SpinMotors( double upperSpeed, double lowerSpeed ) override;
    bool isCenterBroken( ) override;
    bool isEndBroken( ) override;
private:
    //  motor;
};