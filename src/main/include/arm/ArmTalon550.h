#pragma once

#include "ArmIO.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>

#include <rev/SparkMax.h>

class ArmTalon550 : public ArmIO {
public:
    ArmTalon550( );

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

    rev::spark::SparkMax wristMtr;
    // rev::spark::SparkClosedLoopController wristCtrlr = wristMtr.GetClosedLoopController();
    frc::PIDController m_wristPID;
    frc::SimpleMotorFeedforward<units::degrees> m_simpleFF;

    frc::TrapezoidProfile<units::degrees> m_Profile;
    frc::TrapezoidProfile<units::degrees>::State m_Goal;
    frc::TrapezoidProfile<units::degrees>::State m_Setpoint;

    bool isOpenLoopWrist{false};
};
