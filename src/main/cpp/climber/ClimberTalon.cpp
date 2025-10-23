
#include <numbers>

#include "DeviceConstants.h"

#include "climber/ClimberTalon.h"

using namespace device::climber;

ClimberTalon::ClimberTalon( )
    : talon{ deviceIDs::kElevatorID, "" }
{
    ctre::phoenix6::configs::TalonFXConfiguration talonConfigs{};
    talonConfigs.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
    talonConfigs.CurrentLimits.SupplyCurrentLimit = 70_A;
    talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    talonConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    talonConfigs.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
    
    SET_PIDSVGA( talonConfigs.Slot0, kMotionConfig.tuner )
 
    talonConfigs.MotionMagic.MotionMagicCruiseVelocity = kMotionConfig.mp.MaxVelocity;
    talonConfigs.MotionMagic.MotionMagicAcceleration = kMotionConfig.mp.MaxAcceleration;
    talonConfigs.MotionMagic.MotionMagicJerk = kMotionConfig.mp.MaxJerk;

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

void ClimberTalon::Update( Metrics &m ) 
{

    ctre::phoenix6::BaseStatusSignal::RefreshAll( talonPosition, talonVelocity, talonAppliedVolts, talonCurrent );

    m.angle = talonPosition.GetValue();
    m.velocity = talonVelocity.GetValue();
    m.appliedVolts = talonAppliedVolts.GetValue();
    m.current = talonCurrent.GetValue();
}

void ClimberTalon::SetGoal( units::degree_t goal ) 
{
    talon.SetControl( ctre::phoenix6::controls::MotionMagicDutyCycle{ goal } );
}

void ClimberTalon::SetOpenLoop( double percent )
{
    talon.Set( percent );
}

void ClimberTalon::ResetAngle( )
{
   talon.SetPosition( 0_tr );
}