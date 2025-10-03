
#include <numbers>

#include "climber/ClimberSim.h"
#include "DeviceConstants.h"

#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

ClimberSim::ClimberSim()
: motorSim{ device::climber::kDistancePerMotorRev }
{

}

void ClimberSim::Update( Metrics &m ) 
{
   motorSim.Update();

   m.height = motorSim.GetPosition();
   m.velocity = motorSim.GetVelocity();
   m.appliedVolts = motorSim.GetVoltage();
   m.current = motorSim.GetCurrent();

   if( !isHomed && m.height <= -1.0_in ) {
      m.homeSwitchTripped = true;
      isHomed = true;
   } else if( isHomed ) {
      if( units::math::abs(m.height) < 0.5_in  ) {
         m.homeSwitchTripped = true;
      }
   } else {
      m.homeSwitchTripped = false;
   }
}

void ClimberSim::SetGoal( units::inch_t goal ) 
{
   motorSim.SetMotionControl( goal );
}

void ClimberSim::SetOpenLoop( double percent )
{
   motorSim.SetOpenLoop( percent );
}

void ClimberSim::ResetHeight( )
{
   motorSim.SetPosition( 0_in );
}

void ClimberSim::SetRollers( bool enable )
{
    // TODO
}
