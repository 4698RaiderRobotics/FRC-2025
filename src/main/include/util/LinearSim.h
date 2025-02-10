#pragma once

#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/moment_of_inertia.h>

#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include "util/Tuning.h"

class LinearSim {
public:
    using inches_per_rev = units::compound_unit<units::inches, units::inverse<units::turns>>;
    using inches_per_rev_t = units::unit_t<inches_per_rev>;

    LinearSim( inches_per_rev_t mechRatio, double sluggishness=1.0 );
    LinearSim( inches_per_rev_t mechRatio, double sluggishness, const TuningParams &tp );
    LinearSim( inches_per_rev_t mechRatio, double sluggishness, const MotionConfig<units::inch> &mc );

    void SetOpenLoop( double percentOutput );
    void SetMotionControl( units::inch_t goal );
    void SetPosition( units::inch_t position );

    const units::inch_t GetPosition();
    const units::feet_per_second_t GetVelocity();
    const units::volt_t GetVoltage();
    const units::ampere_t GetCurrent();

    void Update();

private:
    frc::sim::DCMotorSim m_motor;

    inline static const units::kilogram_square_meter_t DEFAULT_MOI = 0.0005_kg_sq_m;

    inline static const  MotionConfig<units::inch> DEFAULT_MC = { {1.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0 }, {1_fps, 3_fps_sq, 0_fps_cu}};

    frc::PIDController m_softPID;
    frc::SimpleMotorFeedforward<units::inch> *m_motorFF;
    frc::TrapezoidProfile<units::inch> m_Profile;
    frc::TrapezoidProfile<units::inch>::State m_Goal;
    frc::TrapezoidProfile<units::inch>::State m_Setpoint;
    inches_per_rev_t m_mechRatio;
};

