#pragma once

#include "ElevatorIO.h"

#include <ctre/phoenix6/TalonFX.hpp>

class ElevatorTalon : public ElevatorIO {
public:
    ElevatorTalon( );

    void Update( Metrics &m ) override;

    void ResetPosition( units::inch_t position ) override;
    void SetOpenLoop( double percentOutput ) override;
    void SetGoal( units::inch_t goal ) override;
private:
    ctre::phoenix6::hardware::TalonFX talon;

    ctre::phoenix6::StatusSignal<units::turn_t> talonPosition = talon.GetPosition();
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> talonVelocity = talon.GetVelocity();
    ctre::phoenix6::StatusSignal<units::volt_t> talonAppliedVolts = talon.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<units::ampere_t> talonCurrent = talon.GetSupplyCurrent();
};
