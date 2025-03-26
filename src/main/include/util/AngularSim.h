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
    AngularSim( double gearRatio, double sluggishness, const MotionConfig<units::turn> &mc );

    void SetOpenLoop( double percentOutput );
    void SetMotionControl( units::turn_t goal );
    void SetPosition( units::turn_t position );

    const units::turn_t GetPosition();
    const units::revolutions_per_minute_t GetVelocity();
    const units::volt_t GetVoltage();
    const units::ampere_t GetCurrent();

    void Update();

private:
    frc::sim::DCMotorSim m_motor;

    inline static const units::kilogram_square_meter_t DEFAULT_MOI = 0.0005_kg_sq_m;

    inline static const MotionConfig<units::turn> DEFAULT_MC = { {0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, {3_tps, 12_tr_per_s_sq, 0_tr_per_s_cu}};

    frc::PIDController *m_softPID;
    frc::SimpleMotorFeedforward<units::turn> *m_motorFF;
    frc::TrapezoidProfile<units::turn> m_Profile;
    frc::TrapezoidProfile<units::turn>::State m_Goal;
    frc::TrapezoidProfile<units::turn>::State m_Setpoint;

    bool using_motion_control = false;
};

