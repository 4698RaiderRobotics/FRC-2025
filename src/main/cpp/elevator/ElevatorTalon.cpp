
#include <numbers>

#include "DeviceConstants.h"

#include "elevator/ElevatorTalon.h"

ElevatorTalon::ElevatorTalon( )
    : talon{ deviceIDs::kElevatorID, "" }
{
    ctre::phoenix6::configs::TalonFXConfiguration talonConfigs{};
    talonConfigs.CurrentLimits.SupplyCurrentLimit = 40_A;
    talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    talonConfigs.Slot0.GravityType = ctre::phoenix6::signals::GravityTypeValue::Elevator_Static;
    talonConfigs.Slot0.kS = device::kElevatorS;
    talonConfigs.Slot0.kG = device::kElevatorG;
    talonConfigs.Slot0.kV = device::kElevatorV;
    talonConfigs.Slot0.kA = device::kElevatorA;
    talonConfigs.Slot0.kP = device::kElevatorP;
    talonConfigs.Slot0.kI = device::kElevatorI;
    talonConfigs.Slot0.kD = device::kElevatorD;
    talonConfigs.MotionMagic.MotionMagicCruiseVelocity = device::kElevatorMaxSpeed / device::kElevatorDistancePerMotorRev;
    talonConfigs.MotionMagic.MotionMagicAcceleration = device::kElevatorMaxAcceleration / device::kElevatorDistancePerMotorRev;

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

void ElevatorTalon::Update( Metrics &m ) {

    ctre::phoenix6::BaseStatusSignal::RefreshAll( talonPosition, talonVelocity, talonAppliedVolts, talonCurrent );

    m.height = talonPosition.GetValue() * device::kElevatorDistancePerMotorRev;
    m.velocity = talonVelocity.GetValue() * device::kElevatorDistancePerMotorRev;
    m.appliedVolts = talonAppliedVolts.GetValue();
    m.current = talonCurrent.GetValue();
}

void ElevatorTalon::SetGoal( units::inch_t goal ) {
    units::turn_t turn_goal = goal / device::kElevatorDistancePerMotorRev;
    talon.SetControl( ctre::phoenix6::controls::MotionMagicDutyCycle{ turn_goal } );
}