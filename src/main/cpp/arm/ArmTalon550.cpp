
#include "DeviceConstants.h"
#include "Constants.h"

#include "util/EncOffsets.h"

#include "arm/ArmTalon550.h"

#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;
using namespace device::arm;
using namespace physical::arm;


ArmTalon550::ArmTalon550()
    : elbowMtr{ deviceIDs::kElbowMotorID, "" },
    elbowEncoder{ deviceIDs::kElbowEncoderID, "" },
    wristMtr{ deviceIDs::kWristMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless },
    m_wristPID{ kWristMotionConfig.tuner.kP, kWristMotionConfig.tuner.kI, kWristMotionConfig.tuner.kD },
    m_simpleFF{ kWristMotionConfig.tuner.kS * 1_V, kWristMotionConfig.tuner.kV * 1_V / 1_deg_per_s, kWristMotionConfig.tuner.kA * 1_V / 1_deg_per_s_sq },
    m_Profile{ {kWristMotionConfig.mp.MaxVelocity, kWristMotionConfig.mp.MaxAcceleration} }
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


    SparkMaxConfig config{};
    config
        .Inverted(true)
        .SetIdleMode( SparkMaxConfig::IdleMode::kBrake )
        .SmartCurrentLimit( 30 );
    // config.encoder
    //     .PositionConversionFactor( kWristGearRatio * 360.0 )        // Convert to degrees at intake
    //     .VelocityConversionFactor( kWristGearRatio );               // Convert to RPM of intake
    // config.closedLoop
    //     .SetFeedbackSensor( ClosedLoopConfig::FeedbackSensor::kPrimaryEncoder )
    //     .Pid( kWristMotionConfig.tuner.kP, kWristMotionConfig.tuner.kI, kWristMotionConfig.tuner.kD);
    // config.closedLoop.maxMotion
    //     .MaxVelocity( units::revolutions_per_minute_t{kWristMotionConfig.mp.MaxVelocity}.value() )
    //     .MaxAcceleration( units::revolutions_per_minute_per_second_t{kWristMotionConfig.mp.MaxAcceleration}.value() );
        
    wristMtr.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

void ArmTalon550::Update( Metrics &m )
{
    if( !isOpenLoopWrist ) {
        m_Setpoint = m_Profile.Calculate( 20_ms, m_Setpoint, m_Goal );

        double PIDOut = m_wristPID.Calculate( m.wristPosition.value(), m_Setpoint.position.value() );
        double ffOut = m_simpleFF.Calculate( m_Setpoint.velocity ).value();

        wristMtr.Set( PIDOut + ffOut / 12.0 );
    }
    
    ctre::phoenix6::BaseStatusSignal::RefreshAll( elbowPosition, elbowVelocity, elbowAppliedVolts, elbowCurrent );
    m.elbowPosition = elbowPosition.GetValue();
    m.elbowVelocity = elbowVelocity.GetValue();
    m.elbowAppliedVolts = elbowAppliedVolts.GetValue();
    m.elbowCurrent = elbowCurrent.GetValue();

    m.wristPosition = wristMtr.GetEncoder().GetPosition()  / kWristGearRatio * 360_deg;
    m.wristVelocity = wristMtr.GetEncoder().GetVelocity() / kWristGearRatio * 1_rpm; 
    m.wristAppliedVolts = wristMtr.GetAppliedOutput() * wristMtr.GetBusVoltage() * 1_V;
    m.wristCurrent = wristMtr.GetOutputCurrent() * 1_A;

    EncOffsets::GetInstance().UpdateAngle( "Elbow", m.elbowPosition.value() );
}

void ArmTalon550::SetElbowGoal( units::degree_t goal ) 
{
    elbowMtr.SetControl( ctre::phoenix6::controls::MotionMagicVoltage{ goal } );
}

void ArmTalon550::SetWristPosition( WristPosition pos ) 
{
    isOpenLoopWrist = false;
    switch( pos ) {
    case WristHorizontal:
        // wristCtrlr.SetReference( 0.0, SparkBase::ControlType::kMAXMotionPositionControl );
        m_Goal = { kWristHorizontal, 0_rpm };
        break;
    case WristVertical:
        // wristCtrlr.SetReference( 90.0, SparkBase::ControlType::kMAXMotionPositionControl );
        m_Goal = { kWristVertical, 0_rpm };
        break;
    }
}

void ArmTalon550::ResetWristAngle( units::degree_t position )
{
        // Not sure if this is already converted with PositionConversionFactor()
        // or not.  Luckily it is called with 0.0 
    units::turn_t turns = position * kWristGearRatio;
    wristMtr.GetEncoder().SetPosition( turns.value() );
}

void ArmTalon550::SetWristOpenLoop( double percentOutput )
{
    isOpenLoopWrist = true;
    wristMtr.Set( percentOutput );
}

void ArmTalon550::UpdateElbowOffset() 
{
    ctre::phoenix6::configs::CANcoderConfiguration elbowAbsoluteEncoderConfigs{};
    elbowEncoder.GetConfigurator().Refresh( elbowAbsoluteEncoderConfigs );

    units::turn_t offset = elbowAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset - elbowEncoder.GetAbsolutePosition().GetValue();

    EncOffsets::GetInstance().Set( "Elbow", offset.value() );

    elbowAbsoluteEncoderConfigs.MagnetSensor.MagnetOffset = offset;
    elbowEncoder.GetConfigurator().Apply( elbowAbsoluteEncoderConfigs, 50_ms );
}