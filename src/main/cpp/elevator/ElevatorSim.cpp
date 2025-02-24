
#include <numbers>

#include "elevator/ElevatorSim.h"
#include "DeviceConstants.h"

#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

const MotionConfig<units::inch> simMotion = { {0.05, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0 }, {8_fps, 35_fps_sq, 0_fps_cu}};

ElevatorSim::ElevatorSim()
: motorSim{ device::elevator::kDistancePerMotorRev, 1.0, simMotion }
{
    // Somewhere to home from...
    motorSim.SetPosition( 1_in );
}

void ElevatorSim::Update( Metrics &m ) 
{
    motorSim.Update();

    m.height = motorSim.GetPosition();
    // Don't let the sim go below zero.
    // Set open loop to zero so velocity is zero and homing works.
    if( m.height < 0_in ) {
        m.height = 0_in;
        motorSim.SetPosition( 0_in );
        motorSim.SetOpenLoop( 0 );
    }

    m.velocity = motorSim.GetVelocity();
    m.appliedVolts = motorSim.GetVoltage();
    m.current = motorSim.GetCurrent();
}

void ElevatorSim::ResetPosition( units::inch_t position )
{
    motorSim.SetPosition( position );
}

void ElevatorSim::SetOpenLoop( double percentOutput )
{
    motorSim.SetOpenLoop( percentOutput );
}

void ElevatorSim::SetGoal( units::inch_t goal ) 
{
   motorSim.SetMotionControl( goal );
}