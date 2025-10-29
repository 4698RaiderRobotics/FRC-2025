#pragma once

#include "IntakeIO.h"

#include <frc/DigitalInput.h>
#include <frc/filter/Debouncer.h>

#include <ctre/phoenix6/TalonFXS.hpp>

class IntakeMinion : public IntakeIO {
public:
    IntakeMinion( );
    void Update( Metrics &m ) override;
    
    void SpinMotors( const SpinSpeed &s ) override;
private:
    ctre::phoenix6::hardware::TalonFXS upperMotor;
    ctre::phoenix6::hardware::TalonFXS lowerMotor;

    ctre::phoenix6::StatusSignal<units::turn_t> upperPosition = upperMotor.GetPosition();
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> upperVelocity = upperMotor.GetVelocity();
    ctre::phoenix6::StatusSignal<units::volt_t> upperAppliedVolts = upperMotor.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<units::ampere_t> upperCurrent = upperMotor.GetSupplyCurrent();

    ctre::phoenix6::StatusSignal<units::turn_t> lowerPosition = lowerMotor.GetPosition();
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> lowerVelocity = lowerMotor.GetVelocity();
    ctre::phoenix6::StatusSignal<units::volt_t> lowerAppliedVolts = lowerMotor.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<units::ampere_t> lowerCurrent = lowerMotor.GetSupplyCurrent();

    frc::DigitalInput centerBeamBreak;   /* True when beam is NOT broken */
    frc::DigitalInput endBeamBreak;      /* True when beam is NOT broken */
};