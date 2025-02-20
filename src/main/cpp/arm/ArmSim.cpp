
#include <numbers>

#include "arm/ArmSim.h"
#include "DeviceConstants.h"

#include "util/Utility.h"

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
    elbowSim.Update();
    wristSim.Update();

    m.elbowPosition = elbowSim.GetPosition();
    m.elbowVelocity = elbowSim.GetVelocity();
    m.elbowAppliedVolts = elbowSim.GetVoltage();
    m.elbowCurrent = elbowSim.GetCurrent();

    m.wristPosition = wristSim.GetPosition();
    m.wristVelocity = wristSim.GetVelocity();
    m.wristAppliedVolts = wristSim.GetVoltage();
    m.wristCurrent = wristSim.GetCurrent();

    // Don't let the wrist sim go outside 0-90 degrees.
    // Set open loop to zero so velocity is zero and homing works.
    if( m.wristPosition < 0_deg || m.wristPosition > 90_deg ) {
        m.wristPosition = util::clamp( m.wristPosition, 0_deg, 90_deg );
        wristSim.SetPosition(m.wristPosition );
        wristSim.SetOpenLoop( 0 );
    }
}

void ArmSim::SetElbowGoal( units::degree_t goal ) 
{
    elbowSim.SetMotionControl( goal );
}

void ArmSim::SetWristPosition( WristPosition pos ) 
{
    switch( pos ) {
    case WristHorizontal:
        wristSim.SetOpenLoop( -0.5 );
        break;
    case WristVertical:
        wristSim.SetOpenLoop( 0.5 );
        break;
    }
}

void ArmSim::ResetWristAngle( units::degree_t position )
{
    wristSim.SetPosition( position );
}

void ArmSim::SetWristOpenLoop( double percentOutput )
{
    wristSim.SetOpenLoop( percentOutput );
}
