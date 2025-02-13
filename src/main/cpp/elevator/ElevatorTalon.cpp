
#include <numbers>

#include "DeviceConstants.h"

#include "elevator/ElevatorTalon.h"

using namespace device::elevator;

ElevatorTalon::ElevatorTalon( )
    : talon{ deviceIDs::kElevatorID, "" }
{
    ctre::phoenix6::configs::TalonFXConfiguration talonConfigs{};
    talonConfigs.CurrentLimits.SupplyCurrentLimit = 40_A;
    talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonConfigs.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
    
    SET_PIDSVGA( talonConfigs.Slot0, kMotionConfig.tuner )
 
    talonConfigs.MotionMagic.MotionMagicCruiseVelocity = kMotionConfig.mp.MaxVelocity / kDistancePerMotorRev;
    talonConfigs.MotionMagic.MotionMagicAcceleration = kMotionConfig.mp.MaxAcceleration / kDistancePerMotorRev;
    talonConfigs.MotionMagic.MotionMagicJerk = kMotionConfig.mp.MaxJerk / kDistancePerMotorRev;

    talon.GetConfigurator().Apply(talonConfigs);

    ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll( 
        50_Hz,
        talonPosition,
        talonVelocity, 
        talonAppliedVolts, 
        talonCurrent
    );

    talon.OptimizeBusUtilization();
}

void ElevatorTalon::Update( Metrics &m ) 
{

    ctre::phoenix6::BaseStatusSignal::RefreshAll( talonPosition, talonVelocity, talonAppliedVolts, talonCurrent );

    m.height = talonPosition.GetValue() * kDistancePerMotorRev;
    m.velocity = talonVelocity.GetValue() * kDistancePerMotorRev;
    m.appliedVolts = talonAppliedVolts.GetValue();
    m.current = talonCurrent.GetValue();
}

void ElevatorTalon::SetOpenLoop( double percentOutput )
{
    talon.Set( percentOutput );
}

void ElevatorTalon::SetGoal( units::inch_t goal ) 
{
    units::turn_t turn_goal = goal / kDistancePerMotorRev;
    talon.SetControl( ctre::phoenix6::controls::MotionMagicDutyCycle{ turn_goal } );
}