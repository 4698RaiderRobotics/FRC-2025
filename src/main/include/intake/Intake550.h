#pragma once

#include "IntakeIO.h"

#include <frc/DigitalInput.h>

#include <rev/SparkMax.h>

class Intake550 : public IntakeIO {
public:
    Intake550( );
    void Update( Metrics &m ) override;
    
    void SpinMotors( double upperSpeed, double lowerSpeed ) override;
private:
    rev::spark::SparkMax upperMotor;
    rev::spark::SparkMax lowerMotor;

    frc::DigitalInput centerBeamBreak;   /* True when beam is NOT broken */
    frc::DigitalInput endBeamBreak;      /* True when beam is NOT broken */
    frc::DigitalInput pipeSwitch;        /* True when pipe is contacting switch */
};