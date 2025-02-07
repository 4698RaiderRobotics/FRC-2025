#pragma once

#include <frc2/command/SubsystemBase.h>

class IntakeIO;

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

    frc2::CommandPtr IntakeCoral();
    frc2::CommandPtr EjectCoral();

private:
    IntakeIO *io;
};
