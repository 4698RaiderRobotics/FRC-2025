
#include "DeviceConstants.h"

#include "intake/Intake550.h"

#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;

Intake550::Intake550()
    : upperMotor{ deviceIDs::kIntakeUpperMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless },
    lowerMotor{ deviceIDs::kIntakeLowerMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless },
    centerBeamBreak{ deviceIDs::kIntakeCenterSensorPort },
    endBeamBreak{ deviceIDs::kIntakeEndSensorPort },
    pipeSwitch{ deviceIDs::kIntakePipeSwitchPort }
{

    SparkMaxConfig config{};

    config
        .Inverted(false)
        // .SetIdleMode( SparkMaxConfig::IdleMode::kBrake )
        .SmartCurrentLimit( 30 );
    // config.closedLoop
    //     .SetFeedbackSensor( ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder )
    //     .Pid(1.0, 0.0, 0.0);
        
    upperMotor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    lowerMotor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

void Intake550::Update( Metrics &m )
{
    m.upperPosition = upperMotor.GetEncoder().GetPosition() * 1_tr / device::intake::kGearRatio;
    m.upperVelocity = upperMotor.GetEncoder().GetVelocity() * 1_rpm / device::intake::kGearRatio;
    m.upperAppliedVolts = upperMotor.GetAppliedOutput() * upperMotor.GetBusVoltage() * 1_V;
    m.upperCurrent = upperMotor.GetOutputCurrent() * 1_A;

    m.lowerPosition = lowerMotor.GetEncoder().GetPosition() * 1_tr / device::intake::kGearRatio;
    m.lowerVelocity = lowerMotor.GetEncoder().GetVelocity() * 1_rpm / device::intake::kGearRatio;
    m.lowerAppliedVolts = lowerMotor.GetAppliedOutput() * lowerMotor.GetBusVoltage() * 1_V;
    m.lowerCurrent = lowerMotor.GetOutputCurrent() * 1_A;

    m.centerBeamBroken = !centerBeamBreak.Get();
    m.endBeamBroken = !endBeamBreak.Get();
    m.pipeSwitchTripped = !pipeSwitch.Get();
}

void Intake550::SpinMotors( const SpinSpeed &s )
{
    upperMotor.Set( s.upperSpeed );
    lowerMotor.Set( s.lowerSpeed );
}
