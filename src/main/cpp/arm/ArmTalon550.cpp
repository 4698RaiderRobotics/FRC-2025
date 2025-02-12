
#include "DeviceConstants.h"

#include "arm/ArmTalon550.h"

#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;
using namespace device::arm;

ArmTalon550::ArmTalon550()
    : elbowMtr{ deviceIDs::kElbowMotorID, "" },
    elbowEncoder{ deviceIDs::kElbowEncoderID, "" },
    wristMtr{ deviceIDs::kWristMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless }
{
    ctre::phoenix6::configs::TalonFXConfiguration talonConfigs{};
    talonConfigs.CurrentLimits.SupplyCurrentLimit = 40_A;
    talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonConfigs.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Arm_Cosine;
    
    SET_PIDSVGA( talonConfigs.Slot0, kMotionConfig.tuner )
 
    talonConfigs.MotionMagic.MotionMagicCruiseVelocity = kMotionConfig.mp.MaxVelocity;
    talonConfigs.MotionMagic.MotionMagicAcceleration = kMotionConfig.mp.MaxAcceleration;
    talonConfigs.MotionMagic.MotionMagicJerk = kMotionConfig.mp.MaxJerk;

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
        // .SetIdleMode( SparkMaxConfig::IdleMode::kBrake )
        .SmartCurrentLimit( 40 );
    // config.closedLoop
    //     .SetFeedbackSensor( ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder )
    //     .Pid(1.0, 0.0, 0.0);
        
    wristMtr.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

void ArmTalon550::Update( Metrics &m )
{
    m.wristPosition = wristMtr.GetEncoder().GetPosition() * 1_tr / kGearRatio;
    m.wristVelocity = wristMtr.GetEncoder().GetVelocity() * 1_rpm / kGearRatio;
    m.wristAppliedVolts = wristMtr.GetAppliedOutput() * wristMtr.GetBusVoltage() * 1_V;
    m.wristCurrent = wristMtr.GetOutputCurrent() * 1_A;

}

