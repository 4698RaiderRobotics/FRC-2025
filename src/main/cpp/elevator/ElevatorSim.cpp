
#include <numbers>

#include "elevator/ElevatorSim.h"
#include "DeviceConstants.h"

#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

ElevatorSim::ElevatorSim()
: motorSim{ device::kElevatorDistancePerMotorRev }
{

}

void ElevatorSim::Update( Metrics &m ) {
    motorSim.Update();

    m.height = motorSim.GetPosition();
    m.velocity = motorSim.GetVelocity();
    m.appliedVolts = motorSim.GetVoltage();
    m.current = motorSim.GetCurrent();
}

void ElevatorSim::SetGoal( units::inch_t goal ) {
   motorSim.SetMotionControl(  goal );
}