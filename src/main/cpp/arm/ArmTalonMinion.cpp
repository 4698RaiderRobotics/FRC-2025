
#include "DeviceConstants.h"
#include "Constants.h"

#include "util/EncOffsets.h"

#include "arm/ArmTalonMinion.h"

using namespace device::arm;
using namespace physical::arm;

ArmTalonMinion::ArmTalonMinion()
    : elbowMtr{ deviceIDs::kElbowMotorID, "" },
    elbowEncoder{ deviceIDs::kElbowEncoderID, "" },
    wristMtr{ deviceIDs::kWristMotorID }
{
    EncOffsets::GetInstance().Listen( "Elbow", [this] { UpdateElbowOffset(); } );

    ctre::phoenix6::configs::CANcoderConfiguration elbowAbsoluteEncoderConfigs{};
    elbowAbsoluteEncoderConfigs.MagnetSensor.SensorDirection = ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive;
    elbowAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = EncOffsets::GetInstance().Get("Elbow") * 1_tr;
    elbowEncoder.GetConfigurator().Apply(elbowAbsoluteEncoderConfigs, 50_ms);

    ctre::phoenix6::configs::TalonFXConfiguration talonConfigs{};
    talonConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    talonConfigs.CurrentLimits.SupplyCurrentLimit = 40_A;
    talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    talonConfigs.Feedback.FeedbackRemoteSensorID = deviceIDs::kElbowEncoderID;
    talonConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
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

    ctre::phoenix6::configs::TalonFXSConfiguration configs;

    configs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    configs.CurrentLimits.SupplyCurrentLimit = 30_A;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    configs.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
    
    SET_PIDSVGA( configs.Slot0, kWristMotionConfig.tuner )
 
    configs.MotionMagic.MotionMagicCruiseVelocity = kWristMotionConfig.mp.MaxVelocity;
    configs.MotionMagic.MotionMagicAcceleration = kWristMotionConfig.mp.MaxAcceleration;
    configs.MotionMagic.MotionMagicJerk = kWristMotionConfig.mp.MaxJerk;

    wristMtr.GetConfigurator().Apply(configs);

    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll( 
        50_Hz,
        wristPosition,
        wristVelocity, 
        wristAppliedVolts, 
        wristCurrent
    );
    wristMtr.OptimizeBusUtilization();

}

void ArmTalonMinion::Update( Metrics &m )
{
    ctre::phoenix6::BaseStatusSignal::RefreshAll( 
        elbowPosition, elbowVelocity, elbowAppliedVolts, elbowCurrent,
        wristPosition, wristVelocity, wristAppliedVolts, wristCurrent
    );

    m.elbowPosition = elbowPosition.GetValue();
    m.elbowVelocity = elbowVelocity.GetValue();
    m.elbowAppliedVolts = elbowAppliedVolts.GetValue();
    m.elbowCurrent = elbowCurrent.GetValue();

    m.wristPosition = wristPosition.GetValue()  / kWristGearRatio;
    m.wristVelocity = wristVelocity.GetValue() / kWristGearRatio; 
    m.wristAppliedVolts = wristAppliedVolts.GetValue();
    m.wristCurrent = wristCurrent.GetValue();

    EncOffsets::GetInstance().UpdateAngle( "Elbow", m.elbowPosition.value() );
}

void ArmTalonMinion::SetElbowGoal( units::degree_t goal ) 
{
    elbowMtr.SetControl( ctre::phoenix6::controls::MotionMagicVoltage{ goal } );
}

void ArmTalonMinion::SetWristPosition( WristPosition pos ) 
{
    isOpenLoopWrist = false;
    switch( pos ) {
    case WristHorizontal:
        wristMtr.SetControl( ctre::phoenix6::controls::MotionMagicVoltage{kWristHorizontal} );
        break;
    case WristVertical:
        wristMtr.SetControl( ctre::phoenix6::controls::MotionMagicVoltage{kWristVertical} );
        break;
    }
}

void ArmTalonMinion::ResetWristAngle( units::degree_t position )
{
        // Not sure if this is already converted with PositionConversionFactor()
        // or not.  Luckily it is called with 0.0 
    units::turn_t turns = position * kWristGearRatio;
    wristMtr.SetPosition( turns );
}

void ArmTalonMinion::SetWristOpenLoop( double percentOutput )
{
    isOpenLoopWrist = true;
    wristMtr.Set( percentOutput );
}

void ArmTalonMinion::UpdateElbowOffset() 
{
    ctre::phoenix6::configs::CANcoderConfiguration elbowAbsoluteEncoderConfigs{};
    elbowEncoder.GetConfigurator().Refresh( elbowAbsoluteEncoderConfigs );

    units::turn_t offset = elbowAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset - elbowEncoder.GetAbsolutePosition().GetValue();

    EncOffsets::GetInstance().Set( "Elbow", offset.value() );

    elbowAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = offset;
    elbowEncoder.GetConfigurator().Apply( elbowAbsoluteEncoderConfigs, 50_ms );
}