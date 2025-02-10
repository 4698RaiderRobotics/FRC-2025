
#include "util/LinearSim.h"


LinearSim::LinearSim( inches_per_rev_t mechRatio, double sluggishness ) 
    : LinearSim( mechRatio, sluggishness, DEFAULT_MC )
{

}

LinearSim::LinearSim( inches_per_rev_t mechRatio, double sluggishness, const TuningParams &tp )
    : LinearSim( mechRatio, sluggishness, {tp, DEFAULT_MC.mp} )
{

}

LinearSim::LinearSim( inches_per_rev_t mechRatio, double sluggishness, const MotionConfig<units::inch> &mc )
    : m_motor{ frc::LinearSystemId::DCMotorSystem( frc::DCMotor::KrakenX60(1), DEFAULT_MOI * sluggishness, 1.0), frc::DCMotor::KrakenX60(1) }, 
      m_softPID{ mc.tuner.kP, mc.tuner.kI, mc.tuner.kD },
      m_Profile{ {mc.mp.MaxVelocity, mc.mp.MaxAcceleration} }, 
      m_mechRatio{mechRatio} 
{
    m_motorFF = new frc::SimpleMotorFeedforward<units::inch>{ 
        units::volt_t{mc.tuner.kS},
        units::unit_t<frc::SimpleMotorFeedforward<units::inch>::kv_unit>{mc.tuner.kV}, 
        units::unit_t<frc::SimpleMotorFeedforward<units::inch>::ka_unit>{mc.tuner.kA}
    };
    m_Goal = {MotionParams<units::inch>::Distance_t(0), MotionParams<units::inch>::Velocity_t(0) };
}

void LinearSim::SetOpenLoop( double percentOutput ) { 
    m_motor.SetInputVoltage( percentOutput * 12_V );  
}

void LinearSim::SetMotionControl( units::inch_t goal ) {
    m_Goal.position = goal; 
}

void LinearSim::SetPosition( units::inch_t position ) { 
    m_motor.SetState( (position / m_mechRatio), 0_rad_per_s ); 
}

const units::inch_t LinearSim::GetPosition()
{
    return m_motor.GetAngularPosition() * m_mechRatio;
}

const units::feet_per_second_t LinearSim::GetVelocity()
{
    return m_motor.GetAngularVelocity() * m_mechRatio;
}


const units::volt_t LinearSim::GetVoltage()
{
    return m_motor.GetInputVoltage();
}


const units::ampere_t LinearSim::GetCurrent()
{
    return m_motor.GetCurrentDraw();
}

void LinearSim::Update() {

    m_Setpoint = m_Profile.Calculate( 20_ms, m_Setpoint, m_Goal);

    units::inch_t currPosition = m_motor.GetAngularPosition() * m_mechRatio;
    
    double pidOut = m_softPID.Calculate( currPosition.value(), m_Setpoint.position.value() );
    units::volt_t ffOut = m_motorFF->Calculate( m_Setpoint.velocity );

    units::volt_t inp_volts = pidOut * 12_V + ffOut;
    if( inp_volts < -12_V ) inp_volts = -12_V;
    else if( inp_volts > 12_V ) inp_volts = 12_V;

    m_motor.SetInputVoltage( inp_volts );

    m_motor.Update( 20_ms );
}