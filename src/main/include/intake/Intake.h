#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/button/Trigger.h>

#include "IntakeIO.h"

class Intake : public frc2::SubsystemBase {
public:
    Intake();
    void Periodic() override;

    void SpinIn();
    void SpinOut();
    void ShiftUp();
    void ShiftDown();
    void Stop();
    bool isCenterBroken();
    bool isEndBroken();
    bool isPipeTripped();

    frc2::CommandPtr IntakeCoral();
    frc2::CommandPtr EjectCoralL1();
    frc2::CommandPtr EjectCoralL2_4( bool waitForPipeSwitch );

    frc2::Trigger HasCoralTrigger() {return frc2::Trigger( [this] { return metrics.centerBeamBroken; } ); }

private:
    std::unique_ptr<IntakeIO> io;
    IntakeIO::Metrics metrics;
};
