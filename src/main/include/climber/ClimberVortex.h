#pragma once

#include "ClimberIO.h"

#include "util/TrapezoidPIDFF.h"

#include <frc/DigitalInput.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <rev/SparkFlex.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include <ctre/phoenix6/CANcoder.hpp>


class ClimberVortex : public ClimberIO {
public:
    ClimberVortex( );

    void Update( Metrics &m ) override;
    void SetGoal( units::degree_t goal ) override;
    void SetOpenLoop( double percent ) override;
    void ResetAngle( ) override;
    void SetRollers( bool enable ) override;

    void UpdateClimberOffset();

private:
    rev::spark::SparkFlex flex;

    ctre::phoenix6::hardware::CANcoder climberEncoder;

    frc::DigitalInput cageEngaged;

    TrapezoidPIDFF profileFF;
    // frc::PIDController m_PID;
    // frc::SimpleMotorFeedforward<units::turns> m_simpleFF;

    // frc::TrapezoidProfile<units::turns> m_Profile;
    // frc::TrapezoidProfile<units::turns>::State m_Goal;
    // frc::TrapezoidProfile<units::turns>::State m_Setpoint;

    bool isOpenLoop;

    ctre::phoenix6::hardware::TalonFX rollerTalon;

    ctre::phoenix6::StatusSignal<units::turns_per_second_t> rollerVelocity = rollerTalon.GetVelocity();
    ctre::phoenix6::StatusSignal<units::volt_t> rollerAppliedVolts = rollerTalon.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<units::ampere_t> rollerCurrent = rollerTalon.GetSupplyCurrent();

};
