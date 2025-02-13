
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
    m.wristPosition = wristMtr.GetEncoder().GetPosition() * 1_tr / kElbowGearRatio;
    m.wristVelocity = wristMtr.GetEncoder().GetVelocity() * 1_rpm / kElbowGearRatio;
    m.wristAppliedVolts = wristMtr.GetAppliedOutput() * wristMtr.GetBusVoltage() * 1_V;
    m.wristCurrent = wristMtr.GetOutputCurrent() * 1_A;

}

void ArmTalon550::SetElbowGoal( units::degree_t goal ) 
{
    units::turn_t at_motor_goal = goal * kElbowGearRatio;
    elbowMtr.SetControl( ctre::phoenix6::controls::MotionMagicDutyCycle{ at_motor_goal } );
}

void ArmTalon550::SetWristHorizontal( WristPosition pos ) 
{
    switch( pos ) {
    case WristHorizontal:
        break;
    case WristVertical:
        break;
    }
}
