#pragma once

#include <string>
#include <units/voltage.h>

#include <frc/controller/PIDController.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include "Tuning.h"

class TrapezoidPIDFF {
public:
    struct CalcResult {
        double PIDout;
        units::volt_t FFout;
        frc::TrapezoidProfile<units::turns>::State setpoint;
    };

    using kv_unit_t = units::unit_t<frc::ArmFeedforward::kv_unit>;
    using ka_unit_t = units::unit_t<frc::ArmFeedforward::ka_unit>;

    TrapezoidPIDFF( MotionConfig<units::turns> config );

    void SetGoal( units::turn_t goal );
    units::turn_t GetGoal() { return m_Goal.position; }
    CalcResult Calculate( units::turn_t current_position );
    void EnableContinuousInput( units::turn_t minimumInput, units::turn_t maximumInput );

private:
    frc::PIDController m_PID;
    frc::ArmFeedforward m_FF;

    frc::TrapezoidProfile<units::turns> m_Profile;
    frc::TrapezoidProfile<units::turns>::State m_Goal;
    frc::TrapezoidProfile<units::turns>::State m_Setpoint;
};

class LinearTrapezoidPIDFF {
public:
    struct CalcResult {
        double PIDout;
        units::volt_t FFout;
        frc::TrapezoidProfile<units::meters>::State setpoint;
    };

    using kv_unit_t = units::unit_t<frc::ElevatorFeedforward::kv_unit>;
    using ka_unit_t = units::unit_t<frc::ElevatorFeedforward::ka_unit>;

    LinearTrapezoidPIDFF( MotionConfig<units::meters> config );

    void SetGoal( units::meter_t goal );
    units::meter_t GetGoal() { return m_Goal.position; }
    CalcResult Calculate( units::meter_t current_position );

private:
    frc::PIDController m_PID;
    frc::ElevatorFeedforward m_FF;

    frc::TrapezoidProfile<units::meters> m_Profile;
    frc::TrapezoidProfile<units::meters>::State m_Goal;
    frc::TrapezoidProfile<units::meters>::State m_Setpoint;
};
