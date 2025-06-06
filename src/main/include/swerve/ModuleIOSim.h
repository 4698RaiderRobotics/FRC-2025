#pragma once

#include <queue>

#include <frc/simulation/DCMotorSim.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include "swerve/ModuleIO.h"


class ModuleIOSim : public ModuleIO {
public:
    ModuleIOSim( const ModuleConfigs& configs );

    virtual void UpdateInputs(ModuleIO::Inputs& inputs) override;

    virtual void SetDriveOpenLoop( double percent ) override;

    virtual void SetTurnOpenLoop( double percent ) override;

    virtual void SetDriveWheelVelocity( units::radians_per_second_t velocity ) override;

    virtual void SetTurnPosition( units::radian_t position ) override;

    virtual void SetDriveBrakeMode( bool enable ) override {}

    virtual void SetTurnBrakeMode( bool enable ) override {}

    virtual int GetIndex() override { return index; }
    
private:
    int index;
    frc::sim::DCMotorSim driveSim;
    frc::sim::DCMotorSim turnSim;

    // Use a software PID controller for turn motor.
    frc::PIDController m_turnPIDController;
    // Use a software PID controller and feedforward for the drive motor.
    frc::PIDController m_drivePIDController;
    frc::SimpleMotorFeedforward<units::radian>* m_driveFF;

    units::radian_t turnAbsoluteInitPosition;
    units::volt_t driveAppliedVolts;
    units::volt_t turnAppliedVolts;

    units::radian_t position_Setpt;
    units::radians_per_second_t velocity_Setpt;

    static inline const TuningParams simDrive_PID = { 0.1, 0.0, 0.0, 0.0, 0.0, 0.124, 0.0 };
    static inline const TuningParams simTurn_PID = { 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
};