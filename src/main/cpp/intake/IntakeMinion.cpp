
#include "DeviceConstants.h"

#include "intake/IntakeMinion.h"


IntakeMinion::IntakeMinion()
    : upperMotor{ deviceIDs::kIntakeUpperMotorID },
    lowerMotor{ deviceIDs::kIntakeLowerMotorID },
    centerBeamBreak{ deviceIDs::kIntakeCenterSensorPort },
    endBeamBreak{ deviceIDs::kIntakeEndSensorPort }
{
    ctre::phoenix6::configs::TalonFXSConfiguration configs;

    configs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    configs.CurrentLimits.SupplyCurrentLimit = 40_A;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    upperMotor.GetConfigurator().Apply(configs);
    lowerMotor.GetConfigurator().Apply(configs);
}

void IntakeMinion::Update( Metrics &m )
{
    ctre::phoenix6::BaseStatusSignal::RefreshAll( 
        upperPosition, upperVelocity, upperAppliedVolts, upperCurrent,
        lowerPosition, lowerVelocity, lowerAppliedVolts, lowerCurrent 
    );

    m.upperPosition = upperPosition.GetValue() / device::intake::kGearRatio;
    m.upperVelocity = upperVelocity.GetValue() / device::intake::kGearRatio;
    m.upperAppliedVolts = upperAppliedVolts.GetValue();
    m.upperCurrent = upperCurrent.GetValue();

    m.lowerPosition = lowerPosition.GetValue() / device::intake::kGearRatio;
    m.lowerVelocity = lowerVelocity.GetValue() / device::intake::kGearRatio;
    m.lowerAppliedVolts = lowerAppliedVolts.GetValue();
    m.lowerCurrent = lowerCurrent.GetValue();

    m.centerBeamBroken = !centerBeamBreak.Get();
    m.endBeamBroken = !endBeamBreak.Get();
}

void IntakeMinion::SpinMotors( const SpinSpeed &s )
{
    upperMotor.Set( s.upperSpeed );
    lowerMotor.Set( s.lowerSpeed );
}
