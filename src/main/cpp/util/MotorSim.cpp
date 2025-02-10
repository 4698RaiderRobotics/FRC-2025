
#include "util/MotorSim.h"

MotorSim::MotorSim() : MotorSim( 1.0, 1.0, DEFAULT_MC )
{

}

MotorSim::MotorSim( double mechRatio, double sluggishness ) 
    : MotorSim( mechRatio, sluggishness, DEFAULT_MC )
{

}

MotorSim::MotorSim( double mechRatio, double sluggishness, const TuningParams &tp )
    : MotorSim( mechRatio, sluggishness, {tp, DEFAULT_MC.mp} )
{

}


MotorSim::MotorSim( double mechRatio, double sluggishness, const MotionConfig<units::radian> &mc )
    : m_motor{ frc::LinearSystemId::DCMotorSystem( frc::DCMotor::KrakenX60(1), MotorSim::DEFAULT_MOI * sluggishness, mechRatio), frc::DCMotor::KrakenX60(1) }, 
      m_softPID{ mc.tuner.kP, mc.tuner.kI, mc.tuner.kD },
      m_Profile{ {mc.mp.MaxVelocity, mc.mp.MaxAcceleration} }, 
      m_mechRatio{mechRatio} 
{

    m_motorFF = new frc::SimpleMotorFeedforward<units::radian>{ 
        units::volt_t{mc.tuner.kS},
        units::unit_t<frc::SimpleMotorFeedforward<units::radian>::kv_unit>{mc.tuner.kV}, 
        units::unit_t<frc::SimpleMotorFeedforward<units::radian>::ka_unit>{mc.tuner.kA}
    };
    m_Goal = {MotionParams<units::radian>::Distance_t(0), MotionParams<units::radian>::Velocity_t(0) };
}

void MotorSim::SetOpenLoop( double percentOutput ) { 
    m_motor.SetInputVoltage( percentOutput * 12_V );  
}

void MotorSim::SetMotionControl( units::radian_t goal ) {
    m_Goal.position = goal; 
}

void MotorSim::SetPosition( units::radian_t position ) { 
    m_motor.SetState( units::radian_t(position.value() / m_mechRatio), 0_rad_per_s ); 
}

void MotorSim::Update() {

    m_Setpoint = m_Profile.Calculate( 20_ms, m_Setpoint, m_Goal);
    
    double pidOut = m_softPID.Calculate( m_motor.GetAngularPosition().value() * m_mechRatio, m_Setpoint.position.value() * m_mechRatio );
    units::volt_t ffOut = m_motorFF->Calculate( m_Setpoint.velocity );

    m_motor.SetInputVoltage( pidOut * 12_V + ffOut );

    m_motor.Update( 20_ms );
}