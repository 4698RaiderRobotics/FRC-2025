
#include "util/AngularSim.h"

AngularSim::AngularSim() : AngularSim( 1.0, 1.0, DEFAULT_MC )
{

}

AngularSim::AngularSim( double gearRatio, double sluggishness ) 
    : AngularSim( gearRatio, sluggishness, DEFAULT_MC )
{

}

AngularSim::AngularSim( double gearRatio, double sluggishness, const TuningParams &tp )
    : AngularSim( gearRatio, sluggishness, {tp, DEFAULT_MC.mp} )
{

}

AngularSim::AngularSim( double gearRatio, double sluggishness, const MotionConfig<units::radian> &mc )
    : m_motor{ frc::LinearSystemId::DCMotorSystem( frc::DCMotor::KrakenX60(1), DEFAULT_MOI * sluggishness, gearRatio), frc::DCMotor::KrakenX60(1) }, 
      m_softPID{ mc.tuner.kP, mc.tuner.kI, mc.tuner.kD },
      m_Profile{ {mc.mp.MaxVelocity, mc.mp.MaxAcceleration} }, 
      m_gearRatio{gearRatio} 
{
    m_motorFF = new frc::SimpleMotorFeedforward<units::radian>{ 
        units::volt_t{mc.tuner.kS},
        units::unit_t<frc::SimpleMotorFeedforward<units::radian>::kv_unit>{mc.tuner.kV}, 
        units::unit_t<frc::SimpleMotorFeedforward<units::radian>::ka_unit>{mc.tuner.kA}
    };
    m_Goal = {MotionParams<units::radian>::Distance_t(0), MotionParams<units::radian>::Velocity_t(0) };
}

void AngularSim::SetOpenLoop( double percentOutput ) { 
    m_motor.SetInputVoltage( percentOutput * 12_V );  
}

void AngularSim::SetMotionControl( units::radian_t goal ) {
    m_Goal.position = goal; 
}

void AngularSim::SetPosition( units::radian_t position ) { 
    m_motor.SetState( units::radian_t(position.value() / m_gearRatio), 0_rad_per_s ); 
}

void AngularSim::Update() {

    m_Setpoint = m_Profile.Calculate( 20_ms, m_Setpoint, m_Goal);

    units::radian_t currPosition = m_motor.GetAngularPosition() * m_gearRatio;

    double pidOut = m_softPID.Calculate( currPosition.value(), m_Setpoint.position.value() );
    units::volt_t ffOut = m_motorFF->Calculate( m_Setpoint.velocity );

    units::volt_t inp_volts = pidOut * 12_V + ffOut;
    if( inp_volts < -12_V ) inp_volts = -12_V;
    else if( inp_volts > 12_V ) inp_volts = 12_V;

    m_motor.SetInputVoltage( pidOut * 12_V + ffOut );

    m_motor.Update( 20_ms );
}