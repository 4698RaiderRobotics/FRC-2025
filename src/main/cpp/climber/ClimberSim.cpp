
#include <numbers>

#include "climber/ClimberSim.h"
#include "DeviceConstants.h"

#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

ClimberSim::ClimberSim()
: motorSim{ device::climber::kGearRatio, 0.5 }
{

}

void ClimberSim::Update( Metrics &m ) 
{
   motorSim.Update();

   m.angle = motorSim.GetPosition();
   m.velocity = motorSim.GetVelocity();
   m.appliedVolts = motorSim.GetVoltage();
   m.current = motorSim.GetCurrent();

   // if( !isHomed && m.angle <= -1.0_in ) {
   //    m.homeSwitchTripped = true;
   //    isHomed = true;
   // } else if( isHomed ) {
   //    if( units::math::abs(m.height) < 0.5_in  ) {
   //       m.homeSwitchTripped = true;
   //    }
   // } else {
   //    m.homeSwitchTripped = false;
   // }
}

void ClimberSim::SetGoal( units::degree_t goal ) 
{
   motorSim.SetMotionControl( goal );
}

void ClimberSim::SetOpenLoop( double percent )
{
   motorSim.SetOpenLoop( percent );
}

void ClimberSim::ResetAngle( )
{
   motorSim.SetPosition( 0_deg );
}

void ClimberSim::SetRollers( bool enable )
{
    // TODO
}

void ClimberSim::EngageRatchet( bool engage )
{
    // Do Nothing...
}
