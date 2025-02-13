
#include <numbers>

#include "arm/ArmSim.h"
#include "DeviceConstants.h"

#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

ArmSim::ArmSim()
: elbowSim{ device::arm::kElbowGearRatio, 1.0 },
wristSim{ device::arm::kWristGearRatio, 0.5 }
{
    // Somewhere to home from...
    wristSim.SetPosition( 10_deg );
}

void ArmSim::Update( Metrics &m ) 
{
    // motorSim.Update();

    // m.height = motorSim.GetPosition();
    // m.velocity = motorSim.GetVelocity();
    // m.appliedVolts = motorSim.GetVoltage();
    // m.current = motorSim.GetCurrent();
}

void ArmSim::SetElbowGoal( units::degree_t goal ) 
{

}

void ArmSim::SetWristHorizontal( WristPosition pos ) 
{
    switch( pos ) {
    case WristHorizontal:
        break;
    case WristVertical:
        break;
    }
}
