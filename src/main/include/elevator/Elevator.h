#pragma once

#include <units/length.h>

#include <frc2/command/SubsystemBase.h>

class ElevatorIO;

class Elevator : public frc2::SubsystemBase {
public:
    Elevator();
    void Periodic() override;

    void SetGoal( units::inch_t goal );
    bool AtGoal();

    frc2::CommandPtr ChangeHeight( units::inch_t elevatorHeightGoal );

private:
    ElevatorIO *io;
};
