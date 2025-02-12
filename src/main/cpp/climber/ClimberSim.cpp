
#include <numbers>

#include "climber/ClimberSim.h"
#include "DeviceConstants.h"

#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

ClimberSim::ClimberSim()
: motorSim{ device::climber::kDistancePerMotorRev }
{

}

void ClimberSim::Update( Metrics &m ) {
    motorSim.Update();

    m.height = motorSim.GetPosition();
    m.velocity = motorSim.GetVelocity();
    m.appliedVolts = motorSim.GetVoltage();
    m.current = motorSim.GetCurrent();
}

void ClimberSim::SetGoal( units::inch_t goal ) {
   motorSim.SetMotionControl( goal );
}