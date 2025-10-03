#pragma once

#include "ClimberIO.h"

#include <frc/DigitalInput.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <rev/SparkFlex.h>
#include <ctre/phoenix6/TalonFX.hpp>

class ClimberVortex : public ClimberIO {
public:
    ClimberVortex( );

    void Update( Metrics &m ) override;

    void SetGoal( units::inch_t goal ) override;
    void SetOpenLoop( double percent ) override;
    void ResetHeight( ) override;
    void SetRollers( bool enable ) override;

private:
    rev::spark::SparkFlex flex;

    frc::DigitalInput climberHome;
    frc::DigitalInput cageEngaged;

    frc::PIDController m_PID;
    frc::SimpleMotorFeedforward<units::meters> m_simpleFF;

    frc::TrapezoidProfile<units::meters> m_Profile;
    frc::TrapezoidProfile<units::meters>::State m_Goal;
    frc::TrapezoidProfile<units::meters>::State m_Setpoint;

    bool isOpenLoop;

    ctre::phoenix6::hardware::TalonFX rollerTalon;

    ctre::phoenix6::StatusSignal<units::turns_per_second_t> rollerVelocity = rollerTalon.GetVelocity();
    ctre::phoenix6::StatusSignal<units::volt_t> rollerAppliedVolts = rollerTalon.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<units::ampere_t> rollerCurrent = rollerTalon.GetSupplyCurrent();

};
