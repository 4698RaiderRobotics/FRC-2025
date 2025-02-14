#pragma once

#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_jerk.h>
#include <units/moment_of_inertia.h>

#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include "util/Tuning.h"

class AngularSim {
public:
    AngularSim( );
    AngularSim( double gearRatio, double sluggishness );
    AngularSim( double gearRatio, double sluggishness, const TuningParams &tp );
    AngularSim( double gearRatio, double sluggishness, const MotionConfig<units::radian> &mc );

    void SetOpenLoop( double percentOutput );
    void SetMotionControl( units::radian_t goal );
    void SetPosition( units::radian_t position );

    const units::radian_t GetPosition();
    const units::radians_per_second_t GetVelocity();
    const units::volt_t GetVoltage();
    const units::ampere_t GetCurrent();

    void Update();

private:
    frc::sim::DCMotorSim m_motor;

    inline static const units::kilogram_square_meter_t DEFAULT_MOI = 0.0005_kg_sq_m;

    inline static const  MotionConfig<units::radian> DEFAULT_MC = { {0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, {1_tps, 3_tr_per_s_sq, 0_tr_per_s_cu}};

    frc::PIDController *m_softPID;
    frc::SimpleMotorFeedforward<units::radian> *m_motorFF;
    frc::TrapezoidProfile<units::radian> m_Profile;
    frc::TrapezoidProfile<units::radian>::State m_Goal;
    frc::TrapezoidProfile<units::radian>::State m_Setpoint;

    bool using_motion_control = false;
};

