
#include <numbers>

#include "DeviceConstants.h"

#include "util/EncOffsets.h"

#include "climber/ClimberVortex.h"

#include <rev/config/SparkFlexConfig.h>

using namespace device::climber;

ClimberVortex::ClimberVortex( )
    : flex{ deviceIDs::kClimberID, rev::spark::SparkLowLevel::MotorType::kBrushless },
    climberEncoder{ deviceIDs::kClimberEncoderID, "" },
    cageEngaged{ deviceIDs::kClimberCageSwitchPort },
    profileFF{ kMotionConfig },
    rollerTalon{ deviceIDs::kClimberRollerID, "" }
{
    EncOffsets::GetInstance().Listen( "Climber", [this] { UpdateClimberOffset(); } );

    ctre::phoenix6::configs::CANcoderConfiguration climberAbsoluteEncoderConfigs{};
    climberAbsoluteEncoderConfigs.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::CounterClockwise_Positive;
    climberAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = EncOffsets::GetInstance().Get("Climber") * 1_tr;
    climberEncoder.GetConfigurator().Apply(climberAbsoluteEncoderConfigs, 50_ms);

    rev::spark::SparkFlexConfig config{};
    config
        .Inverted(true)
        .SetIdleMode( rev::spark::SparkFlexConfig::IdleMode::kBrake )
        .SmartCurrentLimit( 60 );
        
    flex.Configure(config, rev::spark::SparkFlex::ResetMode::kResetSafeParameters, rev::spark::SparkFlex::PersistMode::kPersistParameters);

    ctre::phoenix6::configs::TalonFXConfiguration talonConfigs{};
    talonConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    talonConfigs.CurrentLimits.SupplyCurrentLimit = 60_A;
    talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    rollerTalon.GetConfigurator().Apply(talonConfigs);

}

void ClimberVortex::Update( Metrics &m ) 
{
    m.angle = climberEncoder.GetAbsolutePosition().GetValue();
    m.velocity = climberEncoder.GetVelocity().GetValue();
    m.appliedVolts = flex.GetAppliedOutput() * flex.GetBusVoltage() * 1_V;
    m.current = flex.GetOutputCurrent() * 1_A;
    m.cageSwitchTripped = !cageEngaged.Get();

    ctre::phoenix6::BaseStatusSignal::RefreshAll( 
        rollerVelocity,
        rollerAppliedVolts, 
        rollerCurrent
    );

    m.rollerVelocity = rollerVelocity.GetValue() / kRollerGearRatio;
    m.rollerAppliedVolts = rollerAppliedVolts.GetValue();
    m.rollerCurrent = rollerCurrent.GetValue();

    if( !isOpenLoop ) {
        auto FFresult = profileFF.Calculate( m.angle );
        flex.Set( FFresult.PIDout + FFresult.FFout.value() / 12.0 );
    }

    EncOffsets::GetInstance().UpdateAngle( "Climber", m.angle.value() );
}

void ClimberVortex::SetGoal( units::degree_t goal ) 
{
    isOpenLoop = false;
    profileFF.SetGoal( goal );
}

void ClimberVortex::SetOpenLoop( double percent )
{
    isOpenLoop = true;
    flex.Set( percent );
}

void ClimberVortex::ResetAngle( )
{
    flex.GetEncoder().SetPosition( 0.0 );
}

void ClimberVortex::SetRollers( bool enable )
{
    if( enable ) {
        rollerTalon.Set( 0.75 );
    } else {
        rollerTalon.Set( 0.0 );
    }
}

void ClimberVortex::UpdateClimberOffset() 
{
    ctre::phoenix6::configs::CANcoderConfiguration climberAbsoluteEncoderConfigs{};
    climberEncoder.GetConfigurator().Refresh( climberAbsoluteEncoderConfigs );

    units::turn_t offset = climberAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset - climberEncoder.GetAbsolutePosition().GetValue();

    EncOffsets::GetInstance().Set( "Climber", offset.value() );

    climberAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = offset;
    climberEncoder.GetConfigurator().Apply( climberAbsoluteEncoderConfigs, 50_ms );
}