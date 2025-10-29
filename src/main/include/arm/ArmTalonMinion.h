#pragma once

#include "ArmIO.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

class ArmTalonMinion : public ArmIO {
public:
    ArmTalonMinion( );

    void Update( Metrics &m ) override;

    void SetElbowGoal( units::degree_t goal ) override;
    void SetWristPosition( WristPosition pos ) override;
    void ResetWristAngle( units::degree_t position ) override;
    void SetWristOpenLoop( double percentOutput ) override;
    void UpdateElbowOffset();
private:
    ctre::phoenix6::hardware::TalonFX elbowMtr;
    ctre::phoenix6::hardware::CANcoder elbowEncoder;

    ctre::phoenix6::StatusSignal<units::turn_t> elbowPosition = elbowMtr.GetPosition();
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> elbowVelocity = elbowMtr.GetVelocity();
    ctre::phoenix6::StatusSignal<units::volt_t> elbowAppliedVolts = elbowMtr.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<units::ampere_t> elbowCurrent = elbowMtr.GetSupplyCurrent();

    ctre::phoenix6::hardware::TalonFXS wristMtr;
 
    ctre::phoenix6::StatusSignal<units::turn_t> wristPosition = wristMtr.GetPosition();
    ctre::phoenix6::StatusSignal<units::turns_per_second_t> wristVelocity = wristMtr.GetVelocity();
    ctre::phoenix6::StatusSignal<units::volt_t> wristAppliedVolts = wristMtr.GetMotorVoltage();
    ctre::phoenix6::StatusSignal<units::ampere_t> wristCurrent = wristMtr.GetSupplyCurrent();

    bool isOpenLoopWrist{false};
};
