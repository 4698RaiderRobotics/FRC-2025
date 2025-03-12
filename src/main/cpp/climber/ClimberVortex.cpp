
#include <numbers>

#include "DeviceConstants.h"

#include "climber/ClimberVortex.h"

#include <rev/config/SparkFlexConfig.h>

using namespace device::climber;

ClimberVortex::ClimberVortex( )
    : flex{ deviceIDs::kClimberID, rev::spark::SparkLowLevel::MotorType::kBrushless },
    m_PID{ kMotionConfig.tuner.kP, kMotionConfig.tuner.kI, kMotionConfig.tuner.kD },
    m_simpleFF{ kMotionConfig.tuner.kS * 1_V, kMotionConfig.tuner.kV * 1_V / 1_mps, kMotionConfig.tuner.kA * 1_V / 1_mps_sq },
    m_Profile{ {kMotionConfig.mp.MaxVelocity, kMotionConfig.mp.MaxAcceleration} }
{
    rev::spark::SparkFlexConfig config{};
    config
        .Inverted(true)
        .SetIdleMode( rev::spark::SparkFlexConfig::IdleMode::kBrake )
        .SmartCurrentLimit( 60 );
        
    flex.Configure(config, rev::spark::SparkFlex::ResetMode::kResetSafeParameters, rev::spark::SparkFlex::PersistMode::kPersistParameters);

}

void ClimberVortex::Update( Metrics &m ) 
{
    if( !isOpenLoop ) {
        m_Setpoint = m_Profile.Calculate( 20_ms, m_Setpoint, m_Goal );

        double current_pos = units::meter_t(flex.GetEncoder().GetPosition() * 1_tr * kDistancePerMotorRev).value();
        double PIDOut = m_PID.Calculate( current_pos, m_Setpoint.position.value() );
        double ffOut = m_simpleFF.Calculate( m_Setpoint.velocity ).value();

        flex.Set( PIDOut + ffOut / 12.0 );
    }

    m.height = flex.GetEncoder().GetPosition() * 1_tr * kDistancePerMotorRev;
    m.velocity = flex.GetEncoder().GetVelocity() * 1_rpm * kDistancePerMotorRev;
    m.appliedVolts = flex.GetAppliedOutput() * flex.GetBusVoltage() * 1_V;
    m.current = flex.GetOutputCurrent() * 1_A;

}

void ClimberVortex::SetGoal( units::inch_t goal ) 
{
    isOpenLoop = false;
    m_Goal = { goal, 0_mps };
}

void ClimberVortex::SetOpenLoop( double percent )
{
    isOpenLoop = true;
    flex.Set( percent );
}

void ClimberVortex::ResetHeight( )
{
    flex.GetEncoder().SetPosition( 0.0 );
}
