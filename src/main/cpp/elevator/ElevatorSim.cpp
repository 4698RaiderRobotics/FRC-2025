
#include <numbers>

#include "elevator/ElevatorSim.h"
#include "DeviceConstants.h"

#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>

ElevatorSim::ElevatorSim()
: motorSim{ device::kElevatorGearRatio, 1.0, {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {1_tps, 3_tr_per_s_sq, 0_tr_per_s_cu}} }
{

}

void ElevatorSim::Update( Metrics &m ) {
    motorSim.Update();

    auto mtr = motorSim.GetMotor();

    m.height = mtr.GetAngularPosition() * device::kElevatorGearDiameter * std::numbers::pi / 1_rad;
    m.velocity = mtr.GetAngularVelocity() * device::kElevatorGearDiameter * std::numbers::pi / 1_rad;
    m.appliedVolts = mtr.GetInputVoltage();
    m.current = mtr.GetCurrentDraw();
}

void ElevatorSim::SetGoal( units::inch_t goal ) {
   motorSim.SetMotionControl( 1_rad * goal / device::kElevatorGearDiameter * std::numbers::pi );
}