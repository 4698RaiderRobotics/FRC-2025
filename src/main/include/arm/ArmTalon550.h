#pragma once

#include "ArmIO.h"

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

#include <rev/SparkMax.h>

class ArmTalon550 : public ArmIO {
public:
    ArmTalon550( );

    void Update( Metrics &m ) override;

    void SetElbowGoal( units::degree_t goal ) override;
    void SetWristPosition( WristPosition pos ) override;
    void SetWristAngle( units::degree_t position ) override;
    void SetWristOpenLoop( double percentOutput ) override;
    void UpdateElbowOffset() override;
private:
    ctre::phoenix6::hardware::TalonFX elbowMtr;
    ctre::phoenix6::hardware::CANcoder elbowEncoder;
    rev::spark::SparkMax wristMtr;

    ctre::phoenix6::StatusSignal<units::turn_t> elbowPosition = elbowMtr.GetPosition();
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> elbowVelocity = elbowMtr.GetVelocity();
    ctre::phoenix6::StatusSignal<units::volt_t> elbowAppliedVolts = elbowMtr.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<units::ampere_t> elbowCurrent = elbowMtr.GetSupplyCurrent();
};
