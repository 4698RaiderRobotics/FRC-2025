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
    void SpinOutFast();
    void ShiftUp();
    void ShiftUpSlow();
    void ShiftDown();
    void Stop();
    bool isCenterBroken();
    bool isEndBroken();
    bool isPipeTripped();

    frc2::CommandPtr IntakeAlgae();
    frc2::CommandPtr IntakeCoral();
    frc2::CommandPtr IntakeCoralNoIndex( units::second_t timeout );
    frc2::CommandPtr IndexCoral();
    frc2::CommandPtr EjectCoralL1();
    frc2::CommandPtr EjectCoralL2_4( bool waitForPipeSwitch );
    frc2::CommandPtr EjectCoralL2_4_Fast();
    frc2::CommandPtr StopCmd();

    frc2::Trigger HasCoralTrigger() {return frc2::Trigger( [this] { return metrics.centerBeamBroken; } ); }

private:
    std::unique_ptr<IntakeIO> io;
    IntakeIO::Metrics metrics;

    bool isStopped{true};
};
