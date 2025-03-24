
#include <numbers>

#include "DeviceConstants.h"

#include "elevator/ElevatorTalon.h"

using namespace device::elevator;

ElevatorTalon::ElevatorTalon( )
    : talon{ deviceIDs::kElevatorID, "" }, talon_follower{ deviceIDs::kElevatorID2, "" }
{
    {   // Configure the main talon
        ctre::phoenix6::configs::TalonFXConfiguration talonConfigs{};
        talonConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
        talonConfigs.CurrentLimits.SupplyCurrentLimit = 60_A;
        talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

        talonConfigs.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
        
        SET_PIDSVGA( talonConfigs.Slot0, kMotionConfig.tuner )
    
        talonConfigs.MotionMagic.MotionMagicCruiseVelocity = kMotionConfig.mp.MaxVelocity / kDistancePerMotorRev;
        talonConfigs.MotionMagic.MotionMagicAcceleration = kMotionConfig.mp.MaxAcceleration / kDistancePerMotorRev;
        talonConfigs.MotionMagic.MotionMagicJerk = kMotionConfig.mp.MaxJerk / kDistancePerMotorRev;

        talon.GetConfigurator().Apply(talonConfigs);
    }
    {   // Configure the follower talon
        ctre::phoenix6::configs::TalonFXConfiguration configs{};
        configs.CurrentLimits.SupplyCurrentLimit = 60_A;
        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

        talon_follower.GetConfigurator().Apply(configs);
    }

    talon_follower.SetControl(ctre::phoenix6::controls::Follower{talon.GetDeviceID(), false});

    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll( 
        50_Hz,
        talonPosition,
        talonVelocity, 
        talonAppliedVolts, 
        talonCurrent,
        followerAppliedVolts,
        followerCurrent
    );

    talon.OptimizeBusUtilization();
    talon_follower.OptimizeBusUtilization();
}

void ElevatorTalon::Update( Metrics &m ) 
{

    ctre::phoenix6::BaseStatusSignal::RefreshAll( 
        talonPosition,
        talonVelocity, 
        talonAppliedVolts, 
        talonCurrent,
        followerAppliedVolts,
        followerCurrent
    );

    m.height = talonPosition.GetValue() * kDistancePerMotorRev;
    m.velocity = talonVelocity.GetValue() * kDistancePerMotorRev;
    m.M1AppliedVolts = talonAppliedVolts.GetValue();
    m.M1Current = talonCurrent.GetValue();
    m.M2AppliedVolts = followerAppliedVolts.GetValue();
    m.M2Current = followerCurrent.GetValue();
}

void ElevatorTalon::ResetPosition( units::inch_t position )
{
    units::turn_t turn_pos = position / kDistancePerMotorRev;
    talon.SetPosition( turn_pos );
}

void ElevatorTalon::SetOpenLoop( double percentOutput )
{
    talon.Set( percentOutput );
}

void ElevatorTalon::SetGoal( units::inch_t goal ) 
{
    units::turn_t turn_goal = goal / kDistancePerMotorRev;
    talon.SetControl( ctre::phoenix6::controls::MotionMagicVoltage{ turn_goal } );
}