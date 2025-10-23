#pragma once

#include "ClimberIO.h"

#include <ctre/phoenix6/TalonFX.hpp>

class ClimberTalon : public ClimberIO {
public:
    ClimberTalon( );

    void Update( Metrics &m ) override;

    void SetGoal( units::degree_t goal ) override;
    void SetOpenLoop( double percent ) override;
    void ResetAngle( ) override;

private:
    ctre::phoenix6::hardware::TalonFX talon;

    ctre::phoenix6::StatusSignal<units::turn_t> talonPosition = talon.GetPosition();
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> talonVelocity = talon.GetVelocity();
    ctre::phoenix6::StatusSignal<units::volt_t> talonAppliedVolts = talon.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<units::ampere_t> talonCurrent = talon.GetSupplyCurrent();
};
