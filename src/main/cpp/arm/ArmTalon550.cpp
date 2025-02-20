
#include "DeviceConstants.h"

#include "arm/ArmTalon550.h"

#include <frc/Preferences.h>

#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;
using namespace device::arm;

ArmTalon550::ArmTalon550()
    : elbowMtr{ deviceIDs::kElbowMotorID, "" },
    elbowEncoder{ deviceIDs::kElbowEncoderID, "" },
    wristMtr{ deviceIDs::kWristMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless }
{
    ctre::phoenix6::configs::CANcoderConfiguration elbowAbsoluteEncoderConfigs{};
    elbowAbsoluteEncoderConfigs.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
    elbowAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = frc::Preferences::GetDouble("ElbowOffset") * 1_tr;
    elbowEncoder.GetConfigurator().Apply(elbowAbsoluteEncoderConfigs, 50_ms);

    ctre::phoenix6::configs::TalonFXConfiguration talonConfigs{};
    talonConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    talonConfigs.CurrentLimits.SupplyCurrentLimit = 40_A;
    talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonConfigs.Feedback.RotorToSensorRatio = kElbowGearRatio;

    talonConfigs.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
    
    SET_PIDSVGA( talonConfigs.Slot0, kElbowMotionConfig.tuner )
 
    talonConfigs.MotionMagic.MotionMagicCruiseVelocity = kElbowMotionConfig.mp.MaxVelocity;
    talonConfigs.MotionMagic.MotionMagicAcceleration = kElbowMotionConfig.mp.MaxAcceleration;
    talonConfigs.MotionMagic.MotionMagicJerk = kElbowMotionConfig.mp.MaxJerk;

    elbowMtr.GetConfigurator().Apply(talonConfigs);

    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll( 
        50_Hz,
        elbowPosition,
        elbowVelocity, 
        elbowAppliedVolts, 
        elbowCurrent
    );

    elbowMtr.OptimizeBusUtilization();

    SparkMaxConfig config{};

    config
        .Inverted(false)
        .SetIdleMode( SparkMaxConfig::IdleMode::kBrake )
        .SmartCurrentLimit( 30 );
    // config.closedLoop
    //     .SetFeedbackSensor( ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder )
    //     .Pid(1.0, 0.0, 0.0);
        
    wristMtr.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

void ArmTalon550::Update( Metrics &m )
{

   ctre::phoenix6::BaseStatusSignal::RefreshAll( elbowPosition, elbowVelocity, elbowAppliedVolts, elbowCurrent );
    m.elbowPosition = elbowPosition.GetValue();
    m.elbowVelocity = elbowVelocity.GetValue();
    m.elbowAppliedVolts = elbowAppliedVolts.GetValue();
    m.elbowCurrent = elbowCurrent.GetValue();

    m.wristPosition = wristMtr.GetEncoder().GetPosition() * 1_tr / kElbowGearRatio;
    m.wristVelocity = wristMtr.GetEncoder().GetVelocity() * 1_rpm / kElbowGearRatio;
    m.wristAppliedVolts = wristMtr.GetAppliedOutput() * wristMtr.GetBusVoltage() * 1_V;
    m.wristCurrent = wristMtr.GetOutputCurrent() * 1_A;

}

void ArmTalon550::SetElbowGoal( units::degree_t goal ) 
{
    elbowMtr.SetControl( ctre::phoenix6::controls::MotionMagicDutyCycle{ goal } );
}

void ArmTalon550::SetWristPosition( WristPosition pos ) 
{
    switch( pos ) {
    case WristHorizontal:
        break;
    case WristVertical:
        break;
    }
}

void ArmTalon550::SetWristAngle( units::degree_t position )
{
    units::turn_t turns = position;
    wristMtr.GetEncoder().SetPosition( turns.value() );
}

void ArmTalon550::SetWristOpenLoop( double percentOutput )
{
    wristMtr.Set( percentOutput );
}

void ArmTalon550::UpdateElbowOffset() 
{
    ctre::phoenix6::configs::CANcoderConfiguration elbowAbsoluteEncoderConfigs{};
    elbowEncoder.GetConfigurator().Refresh( elbowAbsoluteEncoderConfigs );

    units::turn_t offset = elbowAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset - elbowEncoder.GetAbsolutePosition().GetValue();

    frc::Preferences::SetDouble("ElbowOffset", offset.value());

    elbowAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = offset;
    elbowEncoder.GetConfigurator().Apply( elbowAbsoluteEncoderConfigs, 50_ms );
}