

#include "util/TrapezoidPIDFF.h"


TrapezoidPIDFF::TrapezoidPIDFF( MotionConfig<units::turns> config )
    : m_PID{ config.tuner.kP, config.tuner.kI, config.tuner.kD },
    m_FF{ units::volt_t(config.tuner.kS), units::volt_t(config.tuner.kG), kv_unit_t(config.tuner.kV), ka_unit_t(config.tuner.kA) },
    m_Profile{ {config.mp.MaxVelocity, config.mp.MaxAcceleration} }
{

}

void TrapezoidPIDFF::SetGoal( units::turn_t goal )
{
    m_Goal = { goal, 0_rpm };
    m_PID.Reset();
}

TrapezoidPIDFF::CalcResult TrapezoidPIDFF::Calculate( units::turn_t current_position )
{
    m_Setpoint.position = current_position;
    units::turns_per_second_t current_velocity = m_Setpoint.velocity;
    m_Setpoint = m_Profile.Calculate( 20_ms, m_Setpoint, m_Goal );

    double PIDOut = m_PID.Calculate( current_position.value(), m_Setpoint.position.value() );
    units::volt_t ffOut = m_FF.Calculate( current_position, current_velocity );

    return {PIDOut, ffOut, m_Setpoint};
}

void TrapezoidPIDFF::EnableContinuousInput( units::turn_t minimumInput, units::turn_t maximumInput )
{
    m_PID.EnableContinuousInput( minimumInput.value(), maximumInput.value() );
    m_PID.Reset();
}

LinearTrapezoidPIDFF::LinearTrapezoidPIDFF( MotionConfig<units::meters> config )
    : m_PID{ config.tuner.kP, config.tuner.kI, config.tuner.kD },
    m_FF{ units::volt_t(config.tuner.kS), units::volt_t(config.tuner.kG), kv_unit_t(config.tuner.kV), ka_unit_t(config.tuner.kA) },
    m_Profile{ {config.mp.MaxVelocity, config.mp.MaxAcceleration} }
{

}

void LinearTrapezoidPIDFF::SetGoal( units::meter_t goal )
{
    m_Goal = { goal, 0_mps };
    m_PID.Reset();
}

LinearTrapezoidPIDFF::CalcResult LinearTrapezoidPIDFF::Calculate( units::meter_t current_position )
{
    units::meters_per_second_t current_velocity = m_Setpoint.velocity;
    m_Setpoint = m_Profile.Calculate( 20_ms, m_Setpoint, m_Goal );

    double PIDOut = m_PID.Calculate( current_position.value(), m_Setpoint.position.value() );
    units::volt_t ffOut = m_FF.Calculate( current_velocity,  m_Setpoint.velocity );

    return {PIDOut, ffOut, m_Setpoint};
}
